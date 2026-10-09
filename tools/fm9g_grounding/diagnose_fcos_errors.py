"""Summarize missed reviewed objects by size and nearby predictions."""

import argparse
from collections import Counter
import json
from pathlib import Path

from evaluate_grounding import box_iou, greedy_matches
from grounding_data import CLASSES


def size_bucket(box, width, height):
    x1, y1, x2, y2 = box
    side = ((x2 - x1) * width * (y2 - y1) * height / 1_000_000) ** 0.5
    if side < 16:
        return "lt16"
    if side < 32:
        return "16to31"
    if side < 64:
        return "32to63"
    return "ge64"


def diagnose(rows, threshold, max_examples=12):
    by_class = {name: {"matched": Counter(), "missed": Counter(),
                       "miss_reasons": Counter(), "examples": []} for name in CLASSES}
    for row in rows:
        truth = row["ground_truth"]
        predictions = [obj for obj in row["prediction"] if obj["score"] >= threshold]
        matched = {i for i, _, _ in greedy_matches(truth, predictions, 0.5, True)}
        for index, obj in enumerate(truth):
            result = by_class[obj["class_name"]]
            bucket = size_bucket(obj["bbox"], row["width"], row["height"])
            result["matched" if index in matched else "missed"][bucket] += 1
            if index in matched:
                continue
            overlaps = sorted(((box_iou(obj["bbox"], candidate["bbox"]), candidate)
                               for candidate in predictions), key=lambda pair: pair[0], reverse=True)
            overlap, candidate = overlaps[0] if overlaps else (0, None)
            if overlap >= 0.5 and candidate["class_name"] != obj["class_name"]:
                reason = "wrong_class"
            elif overlap >= 0.1:
                reason = "poor_localization_or_duplicate"
            else:
                reason = "no_nearby_box"
            result["miss_reasons"][reason] += 1
            if len(result["examples"]) < max_examples:
                result["examples"].append({"image": row["image"], "bbox": obj["bbox"],
                                            "size": bucket, "reason": reason,
                                            "best_iou": round(overlap, 3),
                                            "best_class": candidate["class_name"] if candidate else None})
    return by_class


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--predictions", required=True)
    parser.add_argument("--threshold", type=float, required=True)
    args = parser.parse_args()
    if not 0 <= args.threshold < 1:
        parser.error("threshold must be within [0, 1)")
    rows = [json.loads(line) for line in Path(args.predictions).read_text(
        encoding="utf-8").splitlines() if line.strip()]
    print(json.dumps(diagnose(rows, args.threshold), indent=2))


if __name__ == "__main__":
    main()
