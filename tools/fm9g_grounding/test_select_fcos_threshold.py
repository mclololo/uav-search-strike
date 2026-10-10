import unittest

from select_fcos_threshold import scan


class ThresholdSelectionTests(unittest.TestCase):
    def test_selects_threshold_from_validation_predictions(self):
        box = {"class_name": "red_balloon", "bbox": [100, 100, 200, 200]}
        rows = [
            {"width": 640, "height": 480, "parse_mode": "direct", "seconds": 0.03,
             "ground_truth": [box], "prediction": [{**box, "score": 0.8}]},
            {"width": 640, "height": 480, "parse_mode": "direct", "seconds": 0.03,
             "ground_truth": [], "prediction": [{**box, "score": 0.2}]},
        ]
        result = scan(rows, [0.1, 0.5, 0.9])
        self.assertEqual(0.5, result["best_f1_threshold"])
        self.assertEqual(1, result["results"][1]["tp"])
        self.assertEqual(0, result["results"][1]["fp"])
        self.assertEqual(0, result["results"][1]["empty_false_positive_rate"])


if __name__ == "__main__":
    unittest.main()
