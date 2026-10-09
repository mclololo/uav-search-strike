"""Batch evaluation for FM9G grounding predictions on reviewed annotations."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import statistics

from grounding_data import CLASSES, load_records, sha256
from fm9g_grounding import (add_lora, configure_inference, generate, load_model,
                            merge_lora_for_inference, report, restore_adapter)


def percentile(values, quantile):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1 - fraction) + ordered[upper] * fraction


def validate_objects(payload):
    if not isinstance(payload, dict) or set(payload) != {"objects"}:
        raise ValueError("top-level value must contain only objects")
    if not isinstance(payload["objects"], list):
        raise ValueError("objects must be a list")
    result = []
    for obj in payload["objects"]:
        if not isinstance(obj, dict) or set(obj) != {"class_name", "bbox"}:
            raise ValueError("each object must contain only class_name and bbox")
        if obj["class_name"] not in CLASSES:
            raise ValueError("unknown class_name")
        bbox = obj["bbox"]
        if (not isinstance(bbox, list) or len(bbox) != 4
                or any(type(value) is not int for value in bbox)):
            raise ValueError("bbox must contain four integers")
        x1, y1, x2, y2 = bbox
        if not (0 <= x1 < x2 <= 1000 and 0 <= y1 < y2 <= 1000):
            raise ValueError("bbox must be ordered within 0..1000")
        result.append({"class_name": obj["class_name"], "bbox": bbox})
    return result


def parse_prediction(text):
    stripped = text.strip()
    candidates = [("strict", stripped)]
    fenced = re.fullmatch(r"```(?:json)?\s*(\{.*\})\s*```", stripped,
                          flags=re.IGNORECASE | re.DOTALL)
    if fenced:
        candidates.append(("fenced", fenced.group(1)))
    first, last = stripped.find("{"), stripped.rfind("}")
    if first >= 0 and last > first:
        candidates.append(("embedded", stripped[first:last + 1]))
    errors = []
    seen = set()
    for mode, candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        try:
            return validate_objects(json.loads(candidate)), mode, None
        except (json.JSONDecodeError, ValueError) as error:
            errors.append(f"{mode}: {error}")
    return [], "invalid", "; ".join(errors)


def ground_truth_objects(record):
    width, height = record["width"], record["height"]
    objects = []
    for obj in record["objects"]:
        x1, y1, x2, y2 = obj["bbox"]
        objects.append({"class_name": obj["class_name"], "bbox": [
            round(x1 * 1000 / width), round(y1 * 1000 / height),
            round(x2 * 1000 / width), round(y2 * 1000 / height)]})
    return objects


def box_iou(left, right):
    lx1, ly1, lx2, ly2 = left
    rx1, ry1, rx2, ry2 = right
    intersection = max(0, min(lx2, rx2) - max(lx1, rx1)) * max(
        0, min(ly2, ry2) - max(ly1, ry1))
    left_area = (lx2 - lx1) * (ly2 - ly1)
    right_area = (rx2 - rx1) * (ry2 - ry1)
    union = left_area + right_area - intersection
    return intersection / union if union else 0.0


def greedy_matches(truth, predictions, threshold, require_same_class):
    candidates = []
    for truth_index, expected in enumerate(truth):
        for prediction_index, predicted in enumerate(predictions):
            if require_same_class and expected["class_name"] != predicted["class_name"]:
                continue
            overlap = box_iou(expected["bbox"], predicted["bbox"])
            if overlap >= threshold:
                candidates.append((overlap, truth_index, prediction_index))
    matched_truth, matched_predictions, matches = set(), set(), []
    for overlap, truth_index, prediction_index in sorted(candidates, reverse=True):
        if truth_index in matched_truth or prediction_index in matched_predictions:
            continue
        matched_truth.add(truth_index)
        matched_predictions.add(prediction_index)
        matches.append((truth_index, prediction_index, overlap))
    return matches


def ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def compute_metrics(rows, threshold):
    per_class = {name: {"ground_truth": 0, "predictions": 0, "tp": 0,
                        "fp": 0, "fn": 0} for name in CLASSES}
    parse_modes = Counter()
    confusion = {truth: {prediction: 0 for prediction in CLASSES}
                 for truth in CLASSES}
    all_ious, center_errors, latencies = [], [], []
    empty_images = empty_false_positives = 0
    total_truth = total_predictions = total_tp = 0
    for row in rows:
        truth = row["ground_truth"]
        predictions = row["prediction"]
        parse_modes[row["parse_mode"]] += 1
        latencies.append(row["seconds"])
        if not truth:
            empty_images += 1
            if predictions:
                empty_false_positives += 1
        for name in CLASSES:
            class_truth = [obj for obj in truth if obj["class_name"] == name]
            class_predictions = [obj for obj in predictions if obj["class_name"] == name]
            matches = greedy_matches(class_truth, class_predictions, threshold, True)
            true_positives = len(matches)
            stats = per_class[name]
            stats["ground_truth"] += len(class_truth)
            stats["predictions"] += len(class_predictions)
            stats["tp"] += true_positives
            stats["fp"] += len(class_predictions) - true_positives
            stats["fn"] += len(class_truth) - true_positives
        matches = greedy_matches(truth, predictions, threshold, True)
        total_truth += len(truth)
        total_predictions += len(predictions)
        total_tp += len(matches)
        for truth_index, prediction_index, overlap in matches:
            all_ious.append(overlap)
            expected = truth[truth_index]["bbox"]
            predicted = predictions[prediction_index]["bbox"]
            expected_x = (expected[0] + expected[2]) * row["width"] / 2000
            expected_y = (expected[1] + expected[3]) * row["height"] / 2000
            predicted_x = (predicted[0] + predicted[2]) * row["width"] / 2000
            predicted_y = (predicted[1] + predicted[3]) * row["height"] / 2000
            center_errors.append(math.hypot(predicted_x - expected_x,
                                             predicted_y - expected_y))
        for truth_index, prediction_index, _ in greedy_matches(
                truth, predictions, threshold, False):
            confusion[truth[truth_index]["class_name"]][
                predictions[prediction_index]["class_name"]] += 1
    total_fp = total_predictions - total_tp
    total_fn = total_truth - total_tp
    for stats in per_class.values():
        stats["precision"] = ratio(stats["tp"], stats["tp"] + stats["fp"])
        stats["recall"] = ratio(stats["tp"], stats["tp"] + stats["fn"])
        stats["f1"] = ratio(2 * stats["tp"],
                            2 * stats["tp"] + stats["fp"] + stats["fn"])
    precision = ratio(total_tp, total_tp + total_fp)
    recall = ratio(total_tp, total_tp + total_fn)
    return {
        "metrics_version": 2,
        "samples": len(rows),
        "iou_threshold": threshold,
        "parse_modes": dict(parse_modes),
        "strict_json_rate": ratio(parse_modes["strict"], len(rows)),
        "schema_valid_rate": ratio(len(rows) - parse_modes["invalid"], len(rows)),
        "ground_truth_objects": total_truth,
        "predicted_objects": total_predictions,
        "tp": total_tp,
        "fp": total_fp,
        "fn": total_fn,
        "precision": precision,
        "recall": recall,
        "f1": ratio(2 * total_tp, 2 * total_tp + total_fp + total_fn),
        "matched_iou_mean": statistics.fmean(all_ious) if all_ious else None,
        "matched_iou_median": statistics.median(all_ious) if all_ious else None,
        "center_error_pixels_mean": statistics.fmean(center_errors) if center_errors else None,
        "center_error_pixels_median": statistics.median(center_errors) if center_errors else None,
        "center_error_pixels_p95": percentile(center_errors, 0.95),
        "empty_images": empty_images,
        "empty_false_positive_images": empty_false_positives,
        "empty_false_positive_rate": ratio(empty_false_positives, empty_images),
        "empty_invalid_output_images": sum(
            not row["ground_truth"] and row["parse_mode"] == "invalid" for row in rows),
        "generation_seconds_mean": statistics.fmean(latencies) if latencies else None,
        "generation_seconds_p50": percentile(latencies, 0.50),
        "generation_seconds_p95": percentile(latencies, 0.95),
        "generation_seconds_max": max(latencies) if latencies else None,
        "per_class": per_class,
        "class_confusion_at_iou": confusion,
    }


def load_existing(path):
    if not path.exists():
        return []
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise ValueError(f"{path}:{line_number}: {error}") from error
    return rows


def adapter_hash(adapter):
    if not adapter:
        return None
    return sha256(Path(adapter) / "adapter_model.safetensors")


def evaluate(args):
    output = Path(args.output)
    predictions_path = output / "predictions.jsonl"
    config_path = output / "eval-config.json"
    metrics_path = output / "metrics.json"
    records = load_records(args.annotations, args.data_root, require_reviewed=True)
    if args.limit:
        records = records[:args.limit]
    config = {
        "base": str(Path(args.base).resolve()),
        "base_config_sha256": sha256(Path(args.base) / "config.json"),
        "adapter": str(Path(args.adapter).resolve()) if args.adapter else None,
        "adapter_sha256": adapter_hash(args.adapter),
        "data_root": str(Path(args.data_root).resolve()),
        "annotations": str(Path(args.annotations).resolve()),
        "annotations_sha256": sha256(args.annotations),
        "samples": len(records),
        "limit": args.limit,
        "slices": args.slices,
        "max_new_tokens": args.max_new_tokens,
        "iou_threshold": args.iou_threshold,
    }
    # Inference changes must never be appended to a historical evaluation run.
    runtime = {"inference_version": 2, "merge_lora": args.merge_lora,
               "inference_script_sha256": sha256(Path(__file__).with_name("fm9g_grounding.py"))}
    if not args.recompute_metrics:
        config.update(runtime)
    if args.recompute_metrics and not config_path.is_file():
        raise ValueError("Recomputing requires an existing completed evaluation")
    if output.exists() and not (args.resume or args.recompute_metrics):
        raise ValueError("Output exists; pass --resume only for the exact same evaluation")
    output.mkdir(parents=True, exist_ok=True)
    if config_path.exists():
        prior_config = json.loads(config_path.read_text(encoding="utf-8"))
        # Historical complete predictions can still be rescored without new inference.
        if args.recompute_metrics:
            comparable = {k: v for k, v in prior_config.items() if k not in runtime}
            expected = {k: v for k, v in config.items() if k not in runtime}
        else:
            # A legacy evaluation has no runtime fields. Its completed results
            # may be rescored, but cannot establish merged-LoRA behavior.
            comparable = prior_config
            if all(k not in prior_config for k in runtime):
                if args.merge_lora:
                    raise ValueError("Legacy evaluation cannot verify merged LoRA; use a new output directory")
                expected = {k: v for k, v in config.items() if k not in runtime}
                config = prior_config
            else:
                expected = config
        if comparable != expected:
            raise ValueError("Existing evaluation config differs; use a new output directory")
        if args.recompute_metrics:
            config = prior_config
    else:
        config_path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    rows = load_existing(predictions_path)
    if (config_path.exists() and all(k not in config for k in runtime)
            and len(rows) < len(records) and not args.recompute_metrics):
        raise ValueError("Legacy partial evaluation uses different inference settings; use its original script or a new output directory")
    expected_images = {record["image"] for record in records}
    completed = {row["image"] for row in rows}
    if len(completed) != len(rows) or not completed <= expected_images:
        raise ValueError("Existing predictions contain duplicates or unexpected images")
    prior_metrics = (json.loads(metrics_path.read_text(encoding="utf-8"))
                     if metrics_path.exists() else {})
    peak_allocated = prior_metrics.get("peak_allocated_gib")
    if args.recompute_metrics and len(rows) != len(records):
        raise ValueError("Cannot recompute incomplete evaluation; resume generation first")
    if len(rows) < len(records):
        import torch
        from PIL import Image

        model, processor = load_model(args.base)
        if args.adapter:
            adapter_config = json.loads(
                (Path(args.adapter) / "adapter_config.json").read_text(encoding="utf-8"))
            add_lora(model, adapter_config["r"], adapter_config["lora_alpha"],
                     adapter_config["lora_dropout"], training=False)
            restore_adapter(model, args.adapter)
            if args.merge_lora:
                merge_lora_for_inference(model)
        configure_inference(model)
        first_record = next(record for record in records if record["image"] not in completed)
        with Image.open(Path(args.data_root) / first_record["image"]) as source:
            warmup_image = source.convert("RGB")
        _, warmup_seconds = generate(model, processor, warmup_image, args.slices,
                                     args.max_new_tokens)
        report("eval_warmup", seconds=warmup_seconds)
        with predictions_path.open("a", encoding="utf-8") as stream:
            for index, record in enumerate(records, 1):
                if record["image"] in completed:
                    continue
                with Image.open(Path(args.data_root) / record["image"]) as source:
                    image = source.convert("RGB")
                raw_text, seconds = generate(model, processor, image, args.slices,
                                             args.max_new_tokens)
                prediction, parse_mode, parse_error = parse_prediction(raw_text)
                row = {
                    "image": record["image"],
                    "scene_id": record["scene_id"],
                    "width": record["width"],
                    "height": record["height"],
                    "ground_truth": ground_truth_objects(record),
                    "raw_text": raw_text,
                    "parse_mode": parse_mode,
                    "parse_error": parse_error,
                    "prediction": prediction,
                    "seconds": seconds,
                }
                stream.write(json.dumps(row, ensure_ascii=False) + "\n")
                stream.flush()
                rows.append(row)
                report("eval_sample", index=index, total=len(records),
                       image=record["image"], parse_mode=parse_mode,
                       predictions=len(prediction), seconds=seconds)
        peak_allocated = max(peak_allocated or 0,
                             torch.cuda.max_memory_allocated() / 2**30)
    ordered = {row["image"]: row for row in rows}
    rows = [ordered[record["image"]] for record in records]
    metrics = compute_metrics(rows, args.iou_threshold)
    metrics.update({
        "annotations_sha256": config["annotations_sha256"],
        "adapter_sha256": config["adapter_sha256"],
        "peak_allocated_gib": peak_allocated,
        "metrics_script_sha256": sha256(__file__),
        "inference_config": {k: config[k] for k in runtime if k in config},
    })
    metrics_path.write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    report("EVAL_FINISHED", output=str(output), **metrics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="/home/ubuntu/model/FM9G4B-V")
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--annotations", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--adapter")
    parser.add_argument("--merge-lora", action="store_true")
    parser.add_argument("--slices", type=int, default=2)
    parser.add_argument("--max-new-tokens", type=int, default=192)
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--recompute-metrics", action="store_true",
                        help="Recalculate completed saved predictions without GPU inference")
    args = parser.parse_args()
    if args.merge_lora and not args.adapter:
        parser.error("--merge-lora requires --adapter")
    if args.slices <= 0 or args.max_new_tokens <= 0:
        parser.error("slices and max-new-tokens must be positive")
    if not 0 < args.iou_threshold <= 1:
        parser.error("iou-threshold must be in (0, 1]")
    if args.limit is not None and args.limit <= 0:
        parser.error("limit must be positive")
    evaluate(args)


if __name__ == "__main__":
    main()
