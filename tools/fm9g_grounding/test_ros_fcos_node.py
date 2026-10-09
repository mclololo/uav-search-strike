from types import SimpleNamespace
import importlib.util
import unittest

from ros_fcos_node import image_to_rgb, selected_boxes


class Values(list):
    def tolist(self):
        return list(self)


class RosBridgeTests(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec("numpy") and importlib.util.find_spec("PIL"),
                         "Image decoding requires NumPy and Pillow")
    def test_bgr_with_row_padding_becomes_rgb(self):
        message = SimpleNamespace(encoding="bgr8", width=1, height=1, step=4,
                                  data=bytes([1, 2, 3, 0]))
        self.assertEqual([3, 2, 1], image_to_rgb(message)[0, 0].tolist())

    def test_boxes_clipped_and_classified_without_yolo(self):
        detected = {"boxes": Values([[-2.2, 3, 20, 17], [1, 2, 5, 6]]),
                    "labels": Values([1, 0]), "scores": Values([0.7, 0.2])}
        self.assertEqual([("red_balloon", (0, 3, 10, 17), 0.7)],
                         selected_boxes(detected, 10, 20,
                                        ["square_frame", "red_balloon"], 0.43, 8))


if __name__ == "__main__":
    unittest.main()
