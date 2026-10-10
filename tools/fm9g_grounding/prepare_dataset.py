"""Combine reviewed annotation exports and split by explicitly chosen scenes."""

import argparse
from collections import Counter
import json
from pathlib import Path

from grounding_data import CLASSES, check_splits, load_records, sha256


def prepare(data_root, annotation_dir, output, val_scenes, test_scenes):
    root, annotation_dir, output = map(Path, (data_root, annotation_dir, output))
    if output.exists():
        raise ValueError("Output exists; choose a new dataset version")
    files = sorted(annotation_dir.glob("*.jsonl"))
    if not files:
        raise ValueError("No reviewed .jsonl exports in annotation directory")
    records, seen = [], set()
    for path in files:
        for record in load_records(path, root):
            resolved = (root / record["image"]).resolve()
            if resolved in seen:
                raise ValueError(f"Duplicate image across annotation exports: {record['image']}")
            seen.add(resolved)
            records.append(record)
    scenes = {r["scene_id"] for r in records}
    val_scenes, test_scenes = set(val_scenes), set(test_scenes)
    if not val_scenes or not test_scenes or val_scenes & test_scenes:
        raise ValueError("Choose nonempty, disjoint validation and test scenes")
    unknown = (val_scenes | test_scenes) - scenes
    if unknown:
        raise ValueError(f"Unknown held-out scenes: {sorted(unknown)}. Available: {sorted(scenes)}")
    groups = {name: [] for name in ("train", "val", "test")}
    for record in records:
        split = "val" if record["scene_id"] in val_scenes else "test" if record["scene_id"] in test_scenes else "train"
        if split != "train" and record.get("review_status", "reviewed") != "reviewed":
            raise ValueError("YOLO automatic labels are for training; validation/test labels need independent review")
        groups[split].append(record)
    if any(not group for group in groups.values()):
        raise ValueError("Each split must contain at least one reviewed image")
    check_splits(groups["train"], groups["val"], root)
    check_splits(groups["train"], groups["test"], root)
    check_splits(groups["val"], groups["test"], root)
    summary = {"annotation_hashes": {p.name: sha256(p) for p in files}, "splits": {}, "warnings": []}
    for name, group in groups.items():
        counts = Counter(o["class_name"] for r in group for o in r["objects"])
        summary["splits"][name] = dict(images=len(group), empty_images=sum(not r["objects"] for r in group),
                                      scenes=sorted({r["scene_id"] for r in group}),
                                      boxes={cls: counts[cls] for cls in CLASSES})
        missing = [cls for cls in CLASSES if not counts[cls]]
        if missing:
            summary["warnings"].append(f"{name} has no boxes for {missing}")
    output.mkdir(parents=True, exist_ok=False)
    for name, group in groups.items():
        with (output / f"{name}.jsonl").open("w", encoding="utf-8") as stream:
            for record in sorted(group, key=lambda r: (r["scene_id"], r["image"])):
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    (output / "dataset-summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--annotations", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--val-scenes", nargs="+", required=True)
    parser.add_argument("--test-scenes", nargs="+", required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.data_root, args.annotations, args.output,
                             args.val_scenes, args.test_scenes), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
