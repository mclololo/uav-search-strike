import json
from argparse import Namespace
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from evaluate_grounding import compute_metrics, evaluate, parse_prediction


class ParsePredictionTests(unittest.TestCase):
    def test_strict_json(self):
        objects, mode, error = parse_prediction(
            '{"objects":[{"class_name":"red_balloon","bbox":[1,2,3,4]}]}')
        self.assertEqual("strict", mode)
        self.assertIsNone(error)
        self.assertEqual("red_balloon", objects[0]["class_name"])

    def test_fenced_json_is_recoverable_but_not_strict(self):
        objects, mode, error = parse_prediction(
            '```json\n{"objects":[]}\n```')
        self.assertEqual([], objects)
        self.assertEqual("fenced", mode)
        self.assertIsNone(error)

    def test_invalid_schema_is_rejected(self):
        objects, mode, error = parse_prediction(
            '{"objects":[{"class_name":"red_balloon","bbox":[true,2,3,4]}]}')
        self.assertEqual([], objects)
        self.assertEqual("invalid", mode)
        self.assertIn("four integers", error)


class MetricsTests(unittest.TestCase):
    def test_zero_matches_are_zero_f1_not_undefined(self):
        row = {"width": 640, "height": 480, "parse_mode": "strict", "seconds": 1,
               "ground_truth": [{"class_name": "blue_balloon", "bbox": [0, 0, 100, 100]}],
               "prediction": [{"class_name": "purple_balloon", "bbox": [0, 0, 100, 100]}]}
        metrics = compute_metrics([row], 0.5)
        self.assertEqual(0, metrics["f1"])
        self.assertEqual(0, metrics["per_class"]["blue_balloon"]["f1"])
        self.assertEqual(0, metrics["per_class"]["purple_balloon"]["f1"])
        self.assertIsNone(metrics["per_class"]["red_balloon"]["f1"])

    def test_duplicate_prediction_cannot_count_twice(self):
        obj = {"class_name": "red_balloon", "bbox": [0, 0, 100, 100]}
        metrics = compute_metrics([{
            "width": 640, "height": 480, "parse_mode": "strict", "seconds": 1,
            "ground_truth": [obj], "prediction": [obj, obj]}], 0.5)
        self.assertEqual((1, 1, 0), (metrics["tp"], metrics["fp"], metrics["fn"]))

    def test_detection_metrics_and_confusion(self):
        rows = [
            {
                "width": 640, "height": 480, "parse_mode": "strict", "seconds": 1.0,
                "ground_truth": [{"class_name": "red_balloon", "bbox": [100, 100, 200, 200]}],
                "prediction": [{"class_name": "red_balloon", "bbox": [100, 100, 200, 200]}],
            },
            {
                "width": 640, "height": 480, "parse_mode": "fenced", "seconds": 2.0,
                "ground_truth": [],
                "prediction": [{"class_name": "square_frame", "bbox": [100, 100, 200, 200]}],
            },
            {
                "width": 640, "height": 480, "parse_mode": "strict", "seconds": 3.0,
                "ground_truth": [{"class_name": "blue_balloon", "bbox": [100, 100, 200, 200]}],
                "prediction": [{"class_name": "purple_balloon", "bbox": [100, 100, 200, 200]}],
            },
        ]
        metrics = compute_metrics(rows, 0.5)
        self.assertEqual(1, metrics["tp"])
        self.assertEqual(2, metrics["fp"])
        self.assertEqual(1, metrics["fn"])
        self.assertAlmostEqual(1 / 3, metrics["precision"])
        self.assertAlmostEqual(1 / 2, metrics["recall"])
        self.assertEqual(1, metrics["empty_false_positive_images"])
        self.assertEqual(1, metrics["class_confusion_at_iou"]["blue_balloon"]["purple_balloon"])
        self.assertEqual(2 / 3, metrics["strict_json_rate"])
        json.dumps(metrics)


class EvaluationConfigTests(unittest.TestCase):
    def test_legacy_results_cannot_masquerade_as_merged_lora(self):
        with TemporaryDirectory() as directory:
            output = Path(directory)
            args = Namespace(base=str(output), data_root=str(output),
                             annotations=str(output / "val.jsonl"),
                             output=str(output), adapter=None, merge_lora=True,
                             limit=None, slices=2, max_new_tokens=192,
                             iou_threshold=0.5, recompute_metrics=False,
                             resume=True)
            prior = {
                "base": str(output.resolve()), "base_config_sha256": "hash",
                "adapter": None, "adapter_sha256": None,
                "data_root": str(output.resolve()),
                "annotations": str((output / "val.jsonl").resolve()),
                "annotations_sha256": "hash", "samples": 0,
                "limit": None, "slices": 2, "max_new_tokens": 192,
                "iou_threshold": 0.5,
            }
            (output / "eval-config.json").write_text(json.dumps(prior), encoding="utf-8")
            with patch("evaluate_grounding.load_records", return_value=[]), \
                    patch("evaluate_grounding.sha256", return_value="hash"):
                with self.assertRaisesRegex(ValueError, "Legacy evaluation"):
                    evaluate(args)


if __name__ == "__main__":
    unittest.main()
