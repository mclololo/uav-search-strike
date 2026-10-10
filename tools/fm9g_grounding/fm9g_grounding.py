"""Single-GPU FM9G4B-V LoRA preparation and supervised grounding training.

Uses the model's native image processor and preserves the FM9GV wrapper's
attribute layout by injecting PEFT into the language model without wrapping it.
"""

import argparse
import gc
import inspect
import json
import math
from pathlib import Path
import random
import time

from grounding_data import PROMPT, answer_for, check_splits, load_records, sha256


def report(event, **values):
    print(json.dumps({"event": event, **values}, ensure_ascii=True), flush=True)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def load_model(base):
    import torch
    from transformers import AutoModel, AutoProcessor
    if not torch.cuda.is_available() or not torch.cuda.is_bf16_supported():
        raise RuntimeError("A CUDA GPU supporting BF16 is required")
    started = time.perf_counter()
    model = AutoModel.from_pretrained(
        base, trust_remote_code=True, local_files_only=True,
        torch_dtype=torch.bfloat16, attn_implementation="sdpa",
    ).eval().cuda()
    processor = AutoProcessor.from_pretrained(
        base, trust_remote_code=True, local_files_only=True,
    )
    model.processor = processor
    report("model_loaded", model=type(model).__name__,
           processor=type(processor).__name__, forward=str(inspect.signature(model.forward)),
           seconds=round(time.perf_counter() - started, 3))
    return model, processor


def add_lora(model, rank, alpha, dropout, *, training=True):
    import torch
    from peft import LoraConfig, inject_adapter_in_model
    suffixes = {"q_a_proj", "q_b_proj", "kv_a_proj_with_mqa", "kv_b_proj", "o_proj"}
    names = [name for name, module in model.llm.named_modules()
             if isinstance(module, torch.nn.Linear) and name.rsplit(".", 1)[-1] in suffixes]
    if not names:
        raise RuntimeError("No FM9G language attention projection modules found")
    model.requires_grad_(False)
    config = LoraConfig(r=rank, lora_alpha=alpha, lora_dropout=dropout,
                        target_modules=names, bias="none")
    inject_adapter_in_model(config, model.llm)
    if training:
        for param in model.parameters():
            if param.requires_grad:
                param.data = param.data.float()
        model.llm.config.use_cache = False
        # Non-reentrant checkpointing supports frozen input embeddings. A gradient
        # hook on the embeddings would conflict with FM9GV's in-place image scatter.
        model.llm.gradient_checkpointing_enable(
            gradient_checkpointing_kwargs={"use_reentrant": False})
    else:
        configure_inference(model)
    report("lora_added", modules=len(names), example_modules=names[:5],
           trainable_parameters=sum(p.numel() for p in model.parameters() if p.requires_grad))
    return config


def configure_inference(model):
    """Configure generation explicitly; FM9GV.chat ignores arbitrary cache kwargs."""
    model.eval()
    model.requires_grad_(False)
    model.llm.gradient_checkpointing_disable()
    model.llm.config.use_cache = True
    model.llm.generation_config.use_cache = True
    model.llm.generation_config.do_sample = False
    model.llm.generation_config.num_beams = 1


def merge_lora_for_inference(model):
    """Merge only this in-memory model; never save over the base checkpoint."""
    from peft.tuners.lora.layer import LoraLayer
    layers = [(name, module) for name, module in model.llm.named_modules()
              if isinstance(module, LoraLayer)]
    if not layers:
        raise ValueError("No LoRA layers to merge")
    for name, module in layers:
        module.merge(safe_merge=True)
        parent_name, _, child = name.rpartition(".")
        parent = model.llm.get_submodule(parent_name) if parent_name else model.llm
        setattr(parent, child, module.get_base_layer())
    configure_inference(model)
    report("lora_merged_in_memory", modules=len(layers))
    return len(layers)


def make_batch(processor, image, answer, max_length, slices, device):
    import torch
    user = {"role": "user", "content": "(<image>./</image>)\n" + PROMPT}
    prefix = processor.tokenizer.apply_chat_template(
        [user], tokenize=False, add_generation_prompt=True)
    complete = processor.tokenizer.apply_chat_template(
        [user, {"role": "assistant", "content": answer}],
        tokenize=False, add_generation_prompt=False)
    prefix_inputs = processor([prefix], [[image]], max_slice_nums=slices,
                              return_tensors="pt")
    inputs = processor([complete], [[image]], max_slice_nums=slices,
                       return_tensors="pt")
    count = prefix_inputs["input_ids"].shape[1]
    ids = inputs["input_ids"]
    if ids.shape[1] > max_length:
        raise ValueError(f"Sample needs {ids.shape[1]} tokens; limit is {max_length}. No truncation applied.")
    if not torch.equal(ids[:, :count], prefix_inputs["input_ids"]):
        raise ValueError("Chat-template prefix differs from full sample; refusing incorrect label mask")
    if count >= ids.shape[1]:
        raise ValueError("No answer tokens")
    inputs.pop("image_sizes")
    inputs = inputs.to(device)
    inputs["input_ids"] = inputs["input_ids"].long()
    mask = inputs["attention_mask"]
    inputs["position_ids"] = (mask.long().cumsum(-1) - 1).clamp_min(0)
    labels = inputs["input_ids"].clone()
    labels[:, :count] = -100
    labels[~mask.bool()] = -100
    for bounds in inputs["image_bound"]:
        for start, end in bounds.tolist():
            if end > count:
                raise ValueError("Image tokens would be included in the supervised answer")
    return inputs, labels


def supervised_loss(model, inputs, labels):
    import torch
    with torch.autocast("cuda", dtype=torch.bfloat16):
        outputs = model(data=inputs, attention_mask=inputs["attention_mask"],
                        labels=labels, use_cache=False, return_dict=True)
    valid = labels[:, 1:].reshape(-1).ne(-100)
    if outputs.loss is None or outputs.loss.numel() != valid.numel():
        raise RuntimeError("Expected FM9G's unreduced next-token cross entropy")
    if not valid.any():
        raise ValueError("No supervised tokens after causal shift")
    loss = outputs.loss.reshape(-1)[valid].mean()
    if not torch.isfinite(loss):
        raise RuntimeError("Non-finite loss")
    return loss, int(valid.sum())


def train_mode(model):
    model.train()
    model.vpm.eval()
    model.resampler.eval()


def save_adapter(model, config, output):
    from peft import get_peft_model_state_dict
    from safetensors.torch import save_file
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    state = get_peft_model_state_dict(model.llm)
    save_file({name: value.detach().cpu().contiguous() for name, value in state.items()},
              str(output / "adapter_model.safetensors"))
    config.save_pretrained(output)


def restore_adapter(model, adapter):
    from peft import set_peft_model_state_dict
    from safetensors.torch import load_file
    state = load_file(str(Path(adapter) / "adapter_model.safetensors"))
    result = set_peft_model_state_dict(model.llm, state)
    if result.unexpected_keys or any("lora_" in k for k in result.missing_keys):
        raise RuntimeError(f"Adapter key mismatch: {result}")


def generate(model, processor, image, slices, max_tokens):
    import torch
    model.eval()
    torch.cuda.synchronize()
    start = time.perf_counter()
    with torch.inference_mode():
        answer = model.chat(
            image=None, msgs=[{"role": "user", "content": [image, PROMPT]}],
            tokenizer=processor.tokenizer, processor=processor,
            sampling=False, stream=False, num_beams=1, repetition_penalty=1.0,
            max_new_tokens=max_tokens, max_slice_nums=slices)
    torch.cuda.synchronize()
    return answer, time.perf_counter() - start


def verify(model, processor, args):
    import torch
    from PIL import Image
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    image = Image.new("RGB", (640, 480), "white")
    image.save(output / "synthetic-empty-smoke.png")
    report("smoke_only", note="Synthetic empty image; no task accuracy claim")
    config = add_lora(model, args.rank, args.alpha, args.dropout)
    batch, labels = make_batch(processor, image, '{"objects":[]}',
                               args.max_length, args.slices, model.device)
    report("batch_ready", token_count=batch["input_ids"].shape[1],
           answer_tokens=int(labels.ne(-100).sum()),
           image_regions=batch["image_bound"][0].shape[0])
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.AdamW(params, lr=args.learning_rate, weight_decay=0.01)
    losses = []
    for step in range(2):
        train_mode(model)
        optimizer.zero_grad(set_to_none=True)
        loss, _ = supervised_loss(model, batch, labels)
        loss.backward()
        if not any(p.grad is not None and torch.count_nonzero(p.grad).item() for p in params):
            raise RuntimeError("All LoRA gradients are zero")
        norm = torch.nn.utils.clip_grad_norm_(params, 1.0, error_if_nonfinite=True)
        optimizer.step()
        losses.append(loss.item())
        report("smoke_step", step=step + 1, loss=loss.item(), grad_norm=norm.item())
    optimizer.zero_grad(set_to_none=True)
    model.eval()
    with torch.no_grad():
        before, _ = supervised_loss(model, batch, labels)
        expected_loss = before.item()
    adapter = output / "adapter-SMOKE-ONLY"
    save_adapter(model, config, adapter)
    # Clear trained values to verify that reload actually restores the adapter.
    with torch.no_grad():
        for param in params:
            param.zero_()
    restore_adapter(model, adapter)
    with torch.no_grad():
        after, _ = supervised_loss(model, batch, labels)
        restored_loss = after.item()
    if not math.isclose(expected_loss, restored_loss, rel_tol=1e-4, abs_tol=1e-5):
        raise RuntimeError("Adapter reload changed the deterministic validation loss")
    del optimizer, batch, labels
    gc.collect()
    torch.cuda.empty_cache()
    prediction, latency = generate(model, processor, image, args.slices, 48)
    result = dict(smoke_only=True, steps=2, losses=losses,
                  loss_before_reload=expected_loss, loss_after_reload=restored_loss,
                  prediction=prediction, generation_seconds=latency,
                  peak_allocated_gib=torch.cuda.max_memory_allocated() / 2**30)
    write_json(output / "smoke-result.json", result)
    report("SMOKE_OK", **result)


def validation_loss(model, processor, records, args):
    import torch
    from PIL import Image
    model.eval()
    total, tokens = 0.0, 0
    with torch.no_grad():
        for record in records:
            with Image.open(Path(args.data_root) / record["image"]) as source:
                image = source.convert("RGB")
            inputs, labels = make_batch(processor, image, answer_for(record),
                                       args.max_length, args.slices, model.device)
            loss, n = supervised_loss(model, inputs, labels)
            total += loss.item() * n
            tokens += n
    return total / tokens


def train(model, processor, args, train_records, val_records):
    import torch
    from PIL import Image
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    config = add_lora(model, args.rank, args.alpha, args.dropout)
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.AdamW(params, lr=args.learning_rate, weight_decay=0.01)
    total_steps = math.ceil(len(train_records) / args.accumulation) * args.epochs
    warmup = max(1, round(total_steps * 0.05))
    metadata = vars(args).copy()
    metadata.update(train_sha256=sha256(args.train), val_sha256=sha256(args.val),
                    base_config_sha256=sha256(Path(args.base) / "config.json"),
                    script_sha256=sha256(__file__), total_steps=total_steps,
                    restart_policy="New run only; optimizer resume is not implemented")
    write_json(output / "train-config.json", metadata)
    (output / "prompt.txt").write_text(PROMPT + "\n", encoding="utf-8")
    rng = random.Random(args.seed)
    global_step, best = 0, float("inf")
    with (output / "metrics.jsonl").open("w", encoding="utf-8") as log:
        for epoch in range(args.epochs):
            order = list(train_records)
            rng.shuffle(order)
            for offset in range(0, len(order), args.accumulation):
                train_mode(model)
                group = order[offset:offset + args.accumulation]
                optimizer.zero_grad(set_to_none=True)
                if global_step < warmup:
                    factor = (global_step + 1) / warmup
                else:
                    progress = (global_step - warmup) / max(1, total_steps - warmup)
                    factor = 0.5 * (1.0 + math.cos(math.pi * progress))
                for param_group in optimizer.param_groups:
                    param_group["lr"] = args.learning_rate * factor
                step_loss = 0.0
                for record in group:
                    with Image.open(Path(args.data_root) / record["image"]) as source:
                        image = source.convert("RGB")
                    inputs, labels = make_batch(processor, image, answer_for(record),
                                               args.max_length, args.slices, model.device)
                    loss, _ = supervised_loss(model, inputs, labels)
                    (loss / len(group)).backward()
                    step_loss += loss.item() / len(group)
                norm = torch.nn.utils.clip_grad_norm_(params, 1.0, error_if_nonfinite=True)
                optimizer.step()
                global_step += 1
                row = dict(epoch=epoch + 1, step=global_step, loss=step_loss,
                           grad_norm=norm.item(), lr=optimizer.param_groups[0]["lr"])
                log.write(json.dumps(row) + "\n")
                log.flush()
                report("train_step", **row)
            optimizer.zero_grad(set_to_none=True)
            val_loss = validation_loss(model, processor, val_records, args)
            checkpoint = output / f"epoch-{epoch + 1:02d}"
            save_adapter(model, config, checkpoint)
            if val_loss < best:
                best = val_loss
                write_json(output / "best.json", dict(adapter=str(checkpoint), val_loss=best))
            row = dict(epoch=epoch + 1, val_loss=val_loss, adapter=str(checkpoint))
            log.write(json.dumps(row) + "\n")
            log.flush()
            report("validation", **row)
    report("TRAIN_FINISHED", best_validation_loss=best,
           note="Task accuracy and closed-loop behavior require separate evaluation")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["probe", "smoke", "validate", "train", "infer"])
    parser.add_argument("--base", default="/home/ubuntu/model/FM9G4B-V")
    parser.add_argument("--data-root")
    parser.add_argument("--train")
    parser.add_argument("--val")
    parser.add_argument("--output")
    parser.add_argument("--image")
    parser.add_argument("--adapter")
    parser.add_argument("--merge-lora", action="store_true",
                        help="infer only: merge adapter into this process, not base files")
    parser.add_argument("--rank", type=int, default=16)
    parser.add_argument("--alpha", type=int, default=32)
    parser.add_argument("--dropout", type=float, default=0.05)
    parser.add_argument("--learning-rate", type=float, default=5e-5)
    parser.add_argument("--epochs", type=int, default=2)
    parser.add_argument("--accumulation", type=int, default=8)
    parser.add_argument("--max-length", type=int, default=2048)
    parser.add_argument("--max-new-tokens", type=int, default=192)
    parser.add_argument("--slices", type=int, default=2)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.merge_lora and (args.mode != "infer" or not args.adapter):
        parser.error("--merge-lora requires infer with --adapter")
    for name in ("rank", "alpha", "epochs", "accumulation", "max_length", "max_new_tokens", "slices"):
        if getattr(args, name) <= 0:
            parser.error(f"{name} must be positive")
    if not 0 <= args.dropout < 1 or not args.learning_rate > 0:
        parser.error("Invalid dropout or learning rate")
    if args.mode in ("smoke", "train"):
        if not args.output or Path(args.output).exists():
            parser.error("--output must name a new directory; existing results are never overwritten")
    train_records = val_records = None
    if args.mode in ("validate", "train"):
        if not all((args.data_root, args.train, args.val)):
            parser.error("--data-root, --train and --val are required")
        train_records = load_records(args.train, args.data_root)
        val_records = load_records(args.val, args.data_root, require_reviewed=True)
        check_splits(train_records, val_records, args.data_root)
        report("DATA_VALID", train=len(train_records), validation=len(val_records))
        if args.mode == "validate":
            return
    if args.mode == "infer" and not args.image:
        parser.error("--image is required")
    import torch
    torch.manual_seed(args.seed)
    torch.set_num_threads(4)
    model, processor = load_model(args.base)
    if args.mode == "probe":
        add_lora(model, args.rank, args.alpha, args.dropout)
        report("MODEL_LOAD_OK")
    elif args.mode == "smoke":
        verify(model, processor, args)
    elif args.mode == "train":
        train(model, processor, args, train_records, val_records)
    else:
        from PIL import Image
        if args.adapter:
            config = json.loads((Path(args.adapter) / "adapter_config.json").read_text())
            add_lora(model, config["r"], config["lora_alpha"], config["lora_dropout"], training=False)
            restore_adapter(model, args.adapter)
            if args.merge_lora:
                merge_lora_for_inference(model)
        configure_inference(model)
        with Image.open(args.image) as source:
            image = source.convert("RGB")
        prediction, latency = generate(model, processor, image, args.slices, args.max_new_tokens)
        report("prediction", text=prediction, seconds=latency,
               note="Raw model output; not validated for ROS control")


if __name__ == "__main__":
    main()
