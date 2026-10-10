"""Data-integrity checks for asynchronous image/detection capture, without ROS."""

import unittest
from types import SimpleNamespace as NS

from capture_pseudo import FramePairs, make_record
from grounding_data import check_record
from export_yolo_labels import automatic_label


def message(stamp=1, frame='camera', objects=None):
    return NS(header=NS(stamp=NS(secs=stamp, nsecs=0), frame_id=frame),
              width=640, height=480, encoding='rgb8', objects=objects or [])


def box(name='red_balloon', score=.9, right=200):
    return NS(class_name=name, score=score, left_top_x=100, left_top_y=100,
              right_bottom_x=right, right_bottom_y=220)


class CaptureTests(unittest.TestCase):
    def test_detection_can_arrive_first(self):
        pairs = FramePairs()
        detection, image = message(objects=[box()]), message()
        pairs.detection(detection, 1)
        pairs.image(image, 1.1)
        self.assertEqual(pairs.ready(1.2), [(image, detection)])
        self.assertEqual(pairs.ready(2), [])

    def test_waits_for_same_frame_instead_of_latest_detection(self):
        pairs = FramePairs()
        image = message(3)
        pairs.image(image, 1)
        pairs.detection(message(2), 1.2)
        pairs.detection(message(3, frame='other_camera'), 1.3)
        self.assertEqual(pairs.ready(2), [])
        self.assertEqual(pairs.ready(3.1), [(image, None)])

    def test_late_prediction_and_missing_frame_are_not_negative_truth(self):
        pairs = FramePairs()
        image, detection = message(4), message(4, objects=[box()])
        pairs.image(image, 1)
        pairs.detection(detection, 3.5)
        self.assertEqual(pairs.ready(3.6), [(image, None)])
        row = make_record(image, None, 'run', 'flight', 0)
        self.assertEqual(row['detection_status'], 'missing')
        with self.assertRaisesRegex(ValueError, 'Unreviewed'):
            check_record(row, '.', check_image=False)

    def test_reset_zero_and_duplicate_timestamps(self):
        pairs = FramePairs()
        with self.assertRaises(ValueError):
            pairs.image(message(0), 1)
        pairs.image(message(10), 1)
        pairs.image(message(10), 3)
        self.assertEqual(len(pairs.images), 1)
        with self.assertRaisesRegex(ValueError, 'backwards'):
            pairs.image(message(9), 4)

    def test_label_geometry_and_review_gate(self):
        image = message()
        row = make_record(image, message(objects=[box(), box('balloon'),
                          box(right=900), box(score=float('nan'))]), 'run', 'flight', 1)
        self.assertEqual(len(row['objects']), 1)
        self.assertEqual(len(row['rejected_predictions']), 3)
        with self.assertRaisesRegex(ValueError, 'Unreviewed'):
            check_record(row, '.', check_image=False)
        row.update(review_status='reviewed', needs_review=False)
        check_record(row, '.', check_image=False)
        with self.assertRaises(ValueError):
            make_record(image, message(2), 'run', 'flight', 1)

    def test_bounded_buffer_and_flush(self):
        pairs = FramePairs(capacity=2)
        for i in range(1, 8):
            pairs.image(message(i), i)
        self.assertEqual(len(pairs.images), 2)
        self.assertEqual(len(pairs.ready(7, flush=True)), 2)

    def test_direct_yolo_export_preserves_boxes_classes_scores(self):
        row = make_record(message(), message(objects=[box()]), 'run', 'flight', 0)
        result = automatic_label(row)
        self.assertEqual(result['objects'], row['objects'])
        self.assertEqual(result['review_status'], 'auto_accepted')
        self.assertIs(result['human_reviewed'], False)
        self.assertEqual(row['review_status'], 'pending')
        check_record(result, '.', check_image=False)

    def test_direct_export_excludes_unknown_and_invalid_frames(self):
        self.assertIsNone(automatic_label(make_record(message(), None, 'run', 'flight', 0)))
        self.assertIsNone(automatic_label(make_record(message(), message(), 'run', 'flight', 0)))
        row = make_record(message(), message(objects=[box(), box(right=900)]), 'run', 'flight', 0)
        self.assertIsNone(automatic_label(row))


if __name__ == '__main__':
    unittest.main()
