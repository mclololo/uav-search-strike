import unittest

from diagnose_fcos_errors import diagnose, size_bucket


class ErrorDiagnosisTests(unittest.TestCase):
    def test_size_uses_original_image_pixels(self):
        self.assertEqual("lt16", size_bucket([0, 0, 20, 20], 640, 480))
        self.assertEqual("32to63", size_bucket([0, 0, 100, 100], 640, 480))

    def test_small_missing_target_is_counted(self):
        row = {"image": "images/example.png", "width": 640, "height": 480,
               "ground_truth": [{"class_name": "blue_balloon", "bbox": [0, 0, 20, 20]}],
               "prediction": []}
        blue = diagnose([row], 0.43)["blue_balloon"]
        self.assertEqual(1, blue["missed"]["lt16"])
        self.assertEqual(1, blue["miss_reasons"]["no_nearby_box"])


if __name__ == "__main__":
    unittest.main()
