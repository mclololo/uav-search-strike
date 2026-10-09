"""Select a detector score threshold using validation predictions only."""

import argparse
import json
from pathlib import Path

from evaluate_grounding import compute_metrics
from grounding_data import CLASSES


def scan(rows, thresholds):
    results = []
    for threshold in thresholds:
        selected = [{**row, "prediction": [obj for obj in row["prediction"]
                                           if obj["score"] >= threshold]} for row in rows]
        metrics = compute_metrics(selected, 0.5)
        results.append({
            "threshold": threshold,
            "precision": metrics["precision"],
            "recall": metrics["recall"],
            "f1": metrics["f1"],
            "tp": metrics["tp"],
            "fp": metrics["fp"],
            "empty_false_positive_rate": metrics["empty_false_positive_rate"],
            "class_confusion_at_iou": metrics["class_confusion_at_iou"],
            "per_class": {name: {
                key: metrics["per_class"][name][key]
                for key in ("precision", "recall", "f1", "tp", "fp")
            } for name in CLASSES},
        })
    best = max(results, key=lambda result: (result["f1"] or 0, result["recall"] or 0))
    return {"samples": len(rows), "best_f1_threshold": best["threshold"],
            "results": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--predictions", required=True)
    parser.add_argument("--thresholds", nargs="+", type=float,
                        default=[round(value / 100, 2) for value in range(10, 96, 5)])
    args = parser.parse_args()
    if not args.thresholds or any(not 0.01 <= value < 1 for value in args.thresholds):
        parser.error("thresholds must be within [0.01, 1)")
    rows = [json.loads(line) for line in Path(args.predictions).read_text(
        encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        parser.error("predictions must not be empty")
    print(json.dumps(scan(rows, args.thresholds), indent=2))


if __name__ == "__main__":
    main()
