"""Compare legacy, inference and merged LoRA; profile stages without changing weights on disk."""
import argparse
import json
from pathlib import Path
import statistics
import torch
from PIL import Image

from fm9g_grounding import (add_lora, configure_inference, generate, load_model,
                            merge_lora_for_inference, restore_adapter)
from grounding_data import load_records, sha256
from evaluate_grounding import percentile, parse_prediction


class Stages:
    """CUDA events plus actual language forward inputs; hooks removed on exit."""
    def __init__(self, model):
        self.model, self.handles, self.events = model, [], []
        self.pending = {}
        self.calls = []

    def __enter__(self):
        for name, module in [("vision", self.model.vpm), ("resampler", self.model.resampler),
                             ("language", self.model.llm)]:
            def before(mod, args, kwargs, name=name):
                start = torch.cuda.Event(enable_timing=True)
                start.record()
                self.pending[name] = start
                if name == "language":
                    value = kwargs.get("input_ids")
                    if value is None:
                        value = kwargs.get("inputs_embeds")
                    self.calls.append({"sequence": int(value.shape[1]) if value is not None else None,
                                       "use_cache": kwargs.get("use_cache"),
                                       "past_present": kwargs.get("past_key_values") is not None})
            def after(mod, args, result, name=name):
                end = torch.cuda.Event(enable_timing=True)
                end.record()
                self.events.append((name, self.pending.pop(name), end))
            self.handles += [module.register_forward_pre_hook(before, with_kwargs=True),
                             module.register_forward_hook(after)]
        return self

    def __exit__(self, *args):
        for handle in self.handles:
            handle.remove()

    def result(self, total):
        torch.cuda.synchronize()
        times = {}
        for name, start, end in self.events:
            times.setdefault(name, []).append(start.elapsed_time(end) / 1000)
        language = times.get("language", [])
        return {"stage_seconds": {k: sum(v) for k, v in times.items()},
                "prefill_seconds": language[0] if language else None,
                "decode_forward_seconds": sum(language[1:]),
                "other_seconds": max(0, total - sum(sum(v) for v in times.values())),
                "language_forward_calls": len(language),
                "first_calls": self.calls[:3],
                "last_call": self.calls[-1] if self.calls else None,
                "note": "instrumented stage timings; use uninstrumented_seconds for latency comparison"}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--base", default="/home/ubuntu/model/FM9G4B-V")
    p.add_argument("--adapter", required=True)
    p.add_argument("--data-root", required=True)
    p.add_argument("--annotations", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--samples", type=int, default=6)
    p.add_argument("--repeats", type=int, default=2)
    p.add_argument("--slices", type=int, default=2)
    args = p.parse_args()
    if args.samples < 1 or args.repeats < 1:
        p.error("samples and repeats must be positive")
    records = load_records(args.annotations, args.data_root, require_reviewed=True)
    # Cover empty frames and each class, then spread samples across the flight.
    chosen = []
    for cls in [None, "square_frame", "red_balloon", "blue_balloon", "purple_balloon"]:
        for i, r in enumerate(records):
            if (not r["objects"] if cls is None else any(o["class_name"] == cls for o in r["objects"])):
                if i not in chosen:
                    chosen.append(i)
                break
    for i in range(0, len(records), max(1, len(records) // args.samples)):
        if i not in chosen:
            chosen.append(i)
    chosen = chosen[:args.samples]
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(4)
    model, processor = load_model(args.base)
    config = json.loads((Path(args.adapter) / "adapter_config.json").read_text())
    add_lora(model, config["r"], config["lora_alpha"], config["lora_dropout"], training=True)
    restore_adapter(model, args.adapter)
    results, reference = {}, {}
    for mode in ["legacy", "inference", "merged"]:
        if mode != "legacy":
            configure_inference(model)
        if mode == "merged":
            merge_lora_for_inference(model)
        rows = []
        with Image.open(Path(args.data_root) / records[chosen[0]]["image"]) as src:
            generate(model, processor, src.convert("RGB"), args.slices, 192)
        for i in chosen:
            record = records[i]
            with Image.open(Path(args.data_root) / record["image"]) as src:
                img = src.convert("RGB")
            seconds, outputs = [], []
            for _ in range(args.repeats):
                raw, elapsed = generate(model, processor, img, args.slices, 192)
                seconds.append(elapsed)
                outputs.append(raw)
            with Stages(model) as stages:
                raw_profiled, elapsed_profiled = generate(model, processor, img, args.slices, 192)
            row = {"image": record["image"], "uninstrumented_seconds": seconds,
                   "raw_text": outputs[0], "repeat_output_equal": all(x == outputs[0] for x in outputs),
                   "profile_output_equal": raw_profiled == outputs[0],
                   **stages.result(elapsed_profiled)}
            if mode == "legacy":
                reference[row["image"]] = outputs[0]
            row["same_as_legacy"] = outputs[0] == reference[row["image"]]
            row["parse_mode"] = parse_prediction(outputs[0])[1]
            rows.append(row)
            print(json.dumps({"mode": mode, "image": row["image"], "seconds": statistics.mean(seconds),
                              "same_as_legacy": row["same_as_legacy"], "stages": row["stage_seconds"]}),
                  flush=True)
        times = [v for row in rows for v in row["uninstrumented_seconds"]]
        results[mode] = {"seconds_mean": statistics.mean(times), "seconds_p95": percentile(times, .95),
                         "all_outputs_equal_legacy": all(row["same_as_legacy"] for row in rows),
                         "rows": rows}
        (output / "profile.json").write_text(json.dumps({
            "config": vars(args), "annotations_sha256": sha256(args.annotations),
            "adapter_sha256": sha256(Path(args.adapter) / "adapter_model.safetensors"),
            "results": results}, indent=2) + "\n")
    print("PROFILE_FINISHED", flush=True)


if __name__ == "__main__":
    main()
