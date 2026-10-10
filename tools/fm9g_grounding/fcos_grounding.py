"""Offline FCOS detector on FM9G4B-V spatial vision patches; never publishes ROS."""
import argparse
from collections import OrderedDict
import json
import math
from pathlib import Path
import random
import statistics
import time

import torch
from torch import nn
from PIL import Image
from safetensors.torch import load_file, save_file
from torchvision.models.detection import FCOS
from torchvision.models.detection.anchor_utils import AnchorGenerator
from torchvision.transforms.functional import to_tensor

from evaluate_grounding import compute_metrics, percentile
from fm9g_grounding import load_model, report, write_json
from grounding_data import CLASSES, load_records, sha256


class FM9GSpatialBackbone(nn.Module):
    """Use FM9G's ordered patch tokens before its 64-query resampler."""
    out_channels = 256

    def __init__(self, vpm):
        super().__init__()
        self.vpm = vpm
        self.vpm.requires_grad_(False).eval()
        width = vpm.embed_dim
        self.patch_size = vpm.patch_size
        self.projection = nn.Conv2d(width, self.out_channels, 1)
        self.downsample = nn.ModuleList(
            nn.Conv2d(self.out_channels, self.out_channels, 3, stride=2, padding=1)
            for _ in range(4))

    def forward(self, images):
        height, width = images.shape[-2:]
        patch_h, patch_w = height // self.patch_size, width // self.patch_size
        if patch_h < 1 or patch_w < 1:
            raise ValueError("Image too small for the FM9G vision patch size")
        self.vpm.eval()
        with torch.no_grad():
            tokens = self.vpm(images.to(dtype=next(self.vpm.parameters()).dtype)).last_hidden_state
        expected = patch_h * patch_w
        if tokens.ndim != 3 or tokens.shape[1] != expected:
            raise ValueError(f"Vision tokens {tuple(tokens.shape)} do not match {patch_h}x{patch_w}")
        patch_map = tokens.transpose(1, 2).reshape(images.shape[0], -1, patch_h, patch_w)
        feature = self.projection(patch_map.float())
        levels = OrderedDict([("p3", feature)])
        for level, layer in enumerate(self.downsample, 4):
            feature = torch.relu(layer(feature))
            levels[f"p{level}"] = feature
        return levels


def build_detector(base, image_size):
    model, _ = load_model(base)
    vision = model.vpm
    del model.llm, model.resampler, model
    torch.cuda.empty_cache()
    backbone = FM9GSpatialBackbone(vision)
    anchors = AnchorGenerator(
        sizes=((14,), (28,), (56,), (112,), (224,)),
        aspect_ratios=((1.0,),) * 5)
    detector = FCOS(
        backbone, num_classes=len(CLASSES),
        min_size=image_size, max_size=image_size,
        image_mean=[0.5] * 3, image_std=[0.5] * 3,
        anchor_generator=anchors, score_thresh=0.01,
        detections_per_img=100, topk_candidates=300)
    return detector.cuda()


def sample(record, data_root, device):
    with Image.open(Path(data_root) / record["image"]) as source:
        image = to_tensor(source.convert("RGB")).to(device)
    boxes = torch.tensor([obj["bbox"] for obj in record["objects"]],
                         dtype=torch.float32, device=device).reshape(-1, 4)
    labels = torch.tensor([CLASSES.index(obj["class_name"]) for obj in record["objects"]],
                          dtype=torch.int64, device=device)
    return image, {"boxes": boxes, "labels": labels}


def state_without_vpm(model):
    return {name: parameter.detach().cpu().contiguous()
            for name, parameter in model.state_dict().items()
            if not name.startswith("backbone.vpm.")}


def save_head(model, path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=False)
    save_file(state_without_vpm(model), str(path / "head.safetensors"))


def load_head(model, path):
    state = load_file(str(Path(path) / "head.safetensors"))
    result = model.load_state_dict(state, strict=False)
    expected = [name for name in model.state_dict() if name.startswith("backbone.vpm.")]
    if result.unexpected_keys or set(result.missing_keys) != set(expected):
        raise RuntimeError(f"Head state mismatch: {result}")


def prediction_row(record, detected, seconds, threshold):
    width, height = record["width"], record["height"]
    boxes = []
    for box, label, score in zip(detected["boxes"].tolist(),
                                 detected["labels"].tolist(),
                                 detected["scores"].tolist()):
        if score < threshold:
            continue
        x1, y1, x2, y2 = box
        coordinates = [round(max(0, min(1000, x1 / width * 1000))),
                       round(max(0, min(1000, y1 / height * 1000))),
                       round(max(0, min(1000, x2 / width * 1000))),
                       round(max(0, min(1000, y2 / height * 1000)))]
        if coordinates[0] < coordinates[2] and coordinates[1] < coordinates[3]:
            boxes.append({"class_name": CLASSES[label], "bbox": coordinates, "score": score})
    truth = []
    for obj in record["objects"]:
        x1, y1, x2, y2 = obj["bbox"]
        truth.append({"class_name": obj["class_name"], "bbox": [
            round(x1 * 1000 / width), round(y1 * 1000 / height),
            round(x2 * 1000 / width), round(y2 * 1000 / height)]})
    return {"image": record["image"], "width": width, "height": height,
            "ground_truth": truth, "prediction": boxes, "parse_mode": "direct", "seconds": seconds}


def summarize(rows, threshold):
    selected = [{**row, "prediction": [obj for obj in row["prediction"]
                                       if obj["score"] >= threshold]} for row in rows]
    metrics = compute_metrics(selected, 0.5)
    for field in ("parse_modes", "strict_json_rate", "schema_valid_rate"):
        metrics.pop(field)
    for field in list(metrics):
        if field.startswith("generation_seconds_"):
            metrics["inference_seconds_" + field.removeprefix("generation_seconds_")] = metrics.pop(field)
    metrics["score_threshold"] = threshold
    metrics["model"] = "FM9G spatial vision features + torchvision FCOS head"
    return metrics


def evaluate(model, records, data_root, threshold, output=None):
    model.eval()
    rows = []
    image, _ = sample(records[0], data_root, "cuda")
    with torch.inference_mode():
        model([image])
    for index, record in enumerate(records, 1):
        torch.cuda.synchronize()
        start = time.perf_counter()
        image, _ = sample(record, data_root, "cuda")
        with torch.inference_mode():
            detected = model([image])[0]
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - start
        rows.append(prediction_row(record, detected, elapsed, threshold))
        if index % 25 == 0:
            report("detector_eval_progress", index=index, total=len(records))
    if output:
        output = Path(output)
        output.mkdir(parents=True, exist_ok=False)
        with (output / "predictions.jsonl").open("w", encoding="utf-8") as stream:
            for row in rows:
                stream.write(json.dumps(row) + "\n")
        write_json(output / "metrics.json", summarize(rows, threshold))
    return rows


def train(args, model, training, validation):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "train-config.json", {
        **vars(args), "train_sha256": sha256(args.train),
        "val_sha256": sha256(args.val),
        "base_config_sha256": sha256(Path(args.base) / "config.json"),
        "script_sha256": sha256(__file__)})
    optimizer = torch.optim.AdamW(
        [p for p in model.parameters() if p.requires_grad], lr=args.learning_rate,
        weight_decay=0.01)
    rng = random.Random(args.seed)
    best = -1
    with (output / "training.jsonl").open("w", encoding="utf-8") as stream:
        for epoch in range(1, args.epochs + 1):
            indices = list(range(len(training)))
            rng.shuffle(indices)
            model.train()
            total = 0
            for i, index in enumerate(indices, 1):
                img, target = sample(training[index], args.data_root, "cuda")
                optimizer.zero_grad(set_to_none=True)
                losses = model([img], [target])
                loss = sum(losses.values())
                if not torch.isfinite(loss):
                    raise RuntimeError(f"Non-finite detection loss at image {index}")
                loss.backward()
                torch.nn.utils.clip_grad_norm_(
                    [p for p in model.parameters() if p.requires_grad], 1.0)
                optimizer.step()
                total += loss.item()
                if i % 25 == 0:
                    report("detector_train", epoch=epoch, images=i,
                           total=len(training), loss=loss.item())
            checkpoint = output / f"epoch-{epoch:02d}"
            save_head(model, checkpoint)
            rows = evaluate(model, validation, args.data_root, args.score_threshold)
            metrics = summarize(rows, args.score_threshold)
            f1 = metrics["f1"] or 0
            if f1 > best:
                best = f1
                write_json(output / "best.json", {"head": str(checkpoint), "val_f1": f1})
            row = {"epoch": epoch, "loss_mean": total / len(training),
                   "val_f1": f1, "val_recall": metrics["recall"],
                   "val_inference_seconds_mean": metrics["inference_seconds_mean"]}
            stream.write(json.dumps(row) + "\n")
            stream.flush()
            write_json(checkpoint / "val-metrics.json", metrics)
            report("detector_validation", **row)
    report("DETECTOR_TRAIN_FINISHED", best_f1=best)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["probe", "train", "eval"])
    parser.add_argument("--base", default="/home/ubuntu/model/FM9G4B-V")
    parser.add_argument("--data-root", default="data")
    parser.add_argument("--train")
    parser.add_argument("--val")
    parser.add_argument("--annotations")
    parser.add_argument("--head")
    parser.add_argument("--output")
    parser.add_argument("--epochs", type=int, default=2)
    parser.add_argument("--learning-rate", type=float, default=0.0001)
    parser.add_argument("--image-size", type=int, default=448)
    parser.add_argument("--score-threshold", type=float, default=0.2)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.epochs < 1 or args.image_size < 224 or args.image_size > 640:
        parser.error("Invalid epochs or image size")
    if not 0.01 <= args.score_threshold < 1:
        parser.error("score-threshold must be within [0.01, 1)")
    torch.set_num_threads(4)
    torch.manual_seed(args.seed)
    model = build_detector(args.base, args.image_size)
    if args.mode == "probe":
        if args.head:
            load_head(model, args.head)
        model.eval()
        dummy = torch.rand(3, 480, 640, device="cuda")
        with torch.inference_mode():
            features = model.backbone(dummy[None])
            result = model([dummy])[0]
        report("DETECTOR_PROBE_OK", features={k: list(v.shape) for k, v in features.items()},
               boxes=list(result["boxes"].shape), vision_dim=model.backbone.vpm.embed_dim,
               head=args.head)
        return
    if args.mode == "train":
        if not args.train or not args.val or not args.output:
            parser.error("train requires --train, --val, --output")
        train_records = load_records(args.train, args.data_root)
        val_records = load_records(args.val, args.data_root, require_reviewed=True)
        if args.limit:
            train_records = train_records[:args.limit]
            val_records = val_records[:min(args.limit, len(val_records))]
        train(args, model, train_records, val_records)
    else:
        if not args.annotations or not args.head or not args.output:
            parser.error("eval requires --annotations, --head, --output")
        records = load_records(args.annotations, args.data_root, require_reviewed=True)
        load_head(model, args.head)
        if args.limit:
            records = records[:args.limit]
        rows = evaluate(model, records, args.data_root, args.score_threshold, args.output)
        report("DETECTOR_EVAL_FINISHED", output=args.output,
               **summarize(rows, args.score_threshold))


if __name__ == "__main__":
    main()
