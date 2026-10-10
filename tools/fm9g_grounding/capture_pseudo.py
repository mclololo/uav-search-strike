"""Sample raw ROS images and same-stamp /objects predictions for later review.

Read-only subscriber: never starts detection, publishes commands or moves the UAV.
Missing detections are unknown, NOT negative labels. All output needs review.
"""

import argparse
from collections import Counter, OrderedDict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import threading
import time

from capture_frames import decode_rgb
from grounding_data import CLASSES


def frame_key(message):
    stamp = message.header.stamp
    if stamp.secs == 0 and stamp.nsecs == 0:
        raise ValueError("Zero image/detection timestamp cannot establish frame identity")
    return (stamp.secs, stamp.nsecs, message.header.frame_id)


class FramePairs:
    """Bounded cache; caller serializes access. Allows detections to arrive first."""

    def __init__(self, interval=1.0, wait=2.0, capacity=32):
        self.interval, self.wait, self.capacity = interval, wait, capacity
        self.images, self.detections = OrderedDict(), OrderedDict()
        self.last_stamp, self.next_sample = None, 0.0
        self.stats = Counter()

    def image(self, message, now):
        key = frame_key(message)
        if self.last_stamp is not None and key[:2] < self.last_stamp:
            raise ValueError("ROS image time moved backwards; use a new run after restart")
        duplicate = key[:2] == self.last_stamp
        self.last_stamp = key[:2]
        self.stats['images_received'] += 1
        if duplicate or now < self.next_sample:
            return
        if len(self.images) >= self.capacity:
            self.stats['image_queue_full'] += 1
            return
        self.images[key] = (message, now)
        self.next_sample = now + self.interval

    def detection(self, message, now):
        key = frame_key(message)
        self.stats['detections_received'] += 1
        self.detections[key] = (message, now)
        while len(self.detections) > self.capacity * 8:
            self.detections.popitem(last=False)

    def ready(self, now, flush=False):
        output = []
        for key, (message, received) in list(self.images.items()):
            match = self.detections.get(key)
            # Do not attach a very old cached result to a reused timestamp.
            if match and abs(match[1] - received) > self.wait:
                match = None
            if match or flush or now - received >= self.wait:
                output.append((message, match[0] if match else None))
                del self.images[key]
                self.detections.pop(key, None)
        for key, (_, received) in list(self.detections.items()):
            if now - received > self.wait:
                del self.detections[key]
        return output


def make_record(message, detection, run, scene_id, number):
    width, height = message.width, message.height
    key = frame_key(message)
    if detection is not None and frame_key(detection) != key:
        raise ValueError("Refusing to label an image with a different frame's detections")
    objects, rejected = [], []
    for obj in detection.objects if detection is not None else []:
        bbox = [int(obj.left_top_x), int(obj.left_top_y),
                int(obj.right_bottom_x), int(obj.right_bottom_y)]
        score = float(obj.score)
        valid = (obj.class_name in CLASSES and math.isfinite(score) and 0 <= score <= 1
                 and 0 <= bbox[0] < bbox[2] <= width and 0 <= bbox[1] < bbox[3] <= height)
        if not valid:
            rejected.append(dict(class_name=obj.class_name, bbox=bbox,
                                 score=score if math.isfinite(score) else None))
            continue
        objects.append(dict(class_name=obj.class_name, bbox=bbox, score=score,
                            visible=True, source='yolo_ros'))
    return dict(image=f'images/{run}/frame_{number:06d}.png', width=width, height=height,
                scene_id=scene_id, objects=objects, annotation_source='yolo_ros',
                review_status='pending', needs_review=True,
                detection_status='matched' if detection is not None else 'missing',
                rejected_predictions=rejected, stamp_sec=key[0], stamp_nsec=key[1],
                frame_id=key[2], encoding=message.encoding)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', default='/home/ubuntu/fm9g-data/data')
    parser.add_argument('--require-mount', default='/home/ubuntu/fm9g-data')
    parser.add_argument('--run', required=True)
    parser.add_argument('--scene-id', help='Same ID for all chunks of one flight')
    parser.add_argument('--image-topic', default='/rflysim/sensor1/img_rgb')
    parser.add_argument('--objects-topic', default='/objects')
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--seconds', type=float, default=180.0)
    parser.add_argument('--max-frames', type=int, default=180)
    parser.add_argument('--match-wait', type=float, default=2.0)
    parser.add_argument('--first-frame-timeout', type=float, default=30.0)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run):
        parser.error('run may only contain letters, digits, _ and -')
    args.scene_id = args.scene_id or args.run
    if not args.scene_id.strip():
        parser.error('scene-id cannot be empty')
    for value in (args.interval, args.seconds, args.max_frames,
                  args.match_wait, args.first_frame_timeout):
        if not math.isfinite(value) or value <= 0:
            parser.error('Sampling limits must be finite and positive')
    root, mount = Path(args.data_root).resolve(), Path(args.require_mount).resolve()
    if not mount.is_mount() or not root.is_relative_to(mount):
        parser.error('data-root must be on the required, mounted data disk')
    folder = root / 'images' / args.run
    output = root / 'pseudo' / (args.run + '.jsonl')
    if folder.exists() or output.exists():
        parser.error('Run already exists; use a new run name')

    import rospy
    from sensor_msgs.msg import Image
    from common_msgs.msg import Objects
    rospy.init_node('fm9g_pseudo_capture', anonymous=True, disable_signals=True)
    pairs = FramePairs(args.interval, args.match_wait)
    lock, errors = threading.Lock(), []
    first_received = [None]

    def receive_image(message):
        with lock:
            try:
                now = time.monotonic()
                pairs.image(message, now)
                if first_received[0] is None:
                    first_received[0] = now
            except ValueError as error:
                errors.append(str(error))

    def receive_objects(message):
        with lock:
            try:
                pairs.detection(message, time.monotonic())
            except ValueError as error:
                # Bad detections do not turn a sample into a negative label.
                pairs.stats['invalid_detection_headers'] += 1
                rospy.logwarn_throttle(5, str(error))

    subscribers = [rospy.Subscriber(args.image_topic, Image, receive_image,
                                   queue_size=2, buff_size=4 * 1024 * 1024),
                   rospy.Subscriber(args.objects_topic, Objects, receive_objects, queue_size=64)]
    opened, sidecar, count, matched = None, None, 0, 0
    started_waiting = time.monotonic()
    reason = 'finished'
    print('Waiting for raw images and same-stamp YOLO detections; no flight commands.', flush=True)
    try:
        while not rospy.is_shutdown() and count < args.max_frames:
            now = time.monotonic()
            with lock:
                if errors:
                    raise RuntimeError(errors[0])
                first = first_received[0]
                finished = first is not None and now - first >= args.seconds
                samples = pairs.ready(now, flush=finished)
            if first is None and now - started_waiting >= args.first_frame_timeout:
                raise RuntimeError('No images received; start simulation and sensor node first')
            for msg, detection in samples:
                if count >= args.max_frames:
                    break
                if opened is None:
                    folder.mkdir(parents=True, exist_ok=False)
                    output.parent.mkdir(parents=True, exist_ok=True)
                    opened = output.open('x', encoding='utf-8')
                    sidecar = (folder / 'pseudo.jsonl').open('x', encoding='utf-8')
                    metadata = dict(vars(args), started_utc=datetime.now(timezone.utc).isoformat(),
                                    annotation_source='yolo_ros', review_status='pending')
                    # Snapshot teacher settings for interpreting missing/color predictions.
                    metadata['detector_parameters'] = rospy.get_param('/object_detection', {})
                    (folder / 'capture.json').write_text(
                        json.dumps(metadata, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
                record = make_record(msg, detection, args.run, args.scene_id, count)
                image = decode_rgb(msg)
                image.save(root / record['image'])
                line = json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n'
                opened.write(line)
                opened.flush()
                sidecar.write(line)
                sidecar.flush()
                count += 1
                matched += detection is not None
                print(f"SAVED {count}: {record['detection_status']} boxes={len(record['objects'])}", flush=True)
            if finished:
                break
            time.sleep(0.03)
        if rospy.is_shutdown():
            reason = 'ros_shutdown'
    except KeyboardInterrupt:
        reason = 'interrupted'
    except Exception:
        reason = 'error'
        raise
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        if opened:
            opened.close()
            if sidecar:
                sidecar.close()
            summary = dict(images=count, matched_frames=matched, missing_frames=count-matched,
                           review_status='pending', reason=reason, counters=dict(pairs.stats))
            (folder / 'capture-summary.json').write_text(
                json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(f'CAPTURE_FINISHED images={count} matched={matched} pending_review={count} output={output}', flush=True)
    if not count:
        raise RuntimeError('No images saved')


if __name__ == '__main__':
    main()
