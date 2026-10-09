"""Checks that bad geometry and split leakage cannot reach training."""

import tempfile
import unittest
from pathlib import Path

from grounding_data import answer_for, check_record, check_splits


class AnnotationTests(unittest.TestCase):
    def record(self):
        return {"image": "images/frame.png", "width": 640, "height": 480,
                "scene_id": "run_001", "objects": [
                    {"class_name": "red_balloon", "bbox": [160, 120, 320, 240]}]}

    def test_pixel_conversion_and_empty_frame(self):
        record = self.record()
        check_record(record, ".", check_image=False)
        self.assertEqual(answer_for(record),
                         '{"objects":[{"class_name":"red_balloon","bbox":[250,250,500,500]}]}')
        record["objects"] = []
        self.assertEqual(answer_for(record), '{"objects":[]}')

    def test_invalid_geometry_and_class_rejected(self):
        for bbox in [[320, 120, 160, 240], [-1, 0, 10, 10],
                     [0, 0, 641, 480], [0, 0, float("nan"), 2],
                     [0, 0, True, 2], [0, 0, 0.001, 0.001]]:
            record = self.record()
            record["objects"][0]["bbox"] = bbox
            with self.assertRaises(ValueError):
                check_record(record, ".", check_image=False)
        record = self.record()
        record["objects"][0]["class_name"] = "balloon"
        with self.assertRaises(ValueError):
            check_record(record, ".", check_image=False)

    def test_dataset_path_escape_rejected(self):
        record = self.record()
        record["image"] = "../outside.png"
        with self.assertRaises(ValueError):
            check_record(record, ".", check_image=False)

    def test_scene_and_content_leakage_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "one.png").write_bytes(b"same frame bytes")
            (root / "two.png").write_bytes(b"same frame bytes")
            first = {"image": "one.png", "scene_id": "one"}
            second = {"image": "two.png", "scene_id": "two"}
            with self.assertRaisesRegex(ValueError, "Identical image"):
                check_splits([first], [second], root)
            second["scene_id"] = "one"
            with self.assertRaisesRegex(ValueError, "share scenes"):
                check_splits([first], [second], root)


if __name__ == "__main__":
    unittest.main()
