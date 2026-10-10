"""Save bounded, timestamped RGB samples from an existing ROS Image topic.

Does not start simulation, send mission commands, or publish flight controls.
Run with the ROS-enabled demo Python after sourcing the catkin workspace.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import threading
import time


def decode_rgb(msg):
    """Honor encoding and row stride; never infer channels from simulation mode."""
    import numpy as np
    from PIL import Image
    encoding = msg.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    if encoding not in channels:
        raise ValueError(f"Unsupported encoding {msg.encoding!r}; no guessed conversion")
    count = channels[encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * count:
        raise ValueError("Invalid image dimensions or step")
    needed = msg.height * msg.step
    if len(msg.data) < needed:
        raise ValueError("Image data shorter than height * step")
    rows = np.frombuffer(msg.data, dtype=np.uint8, count=needed).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * count].reshape(msg.height, msg.width, count)
    if encoding == "mono8":
        return Image.fromarray(pixels[:, :, 0].copy()).convert("RGB")
    pixels = pixels[:, :, :3]
    if encoding in ("bgr8", "bgra8"):
        pixels = pixels[:, :, ::-1]
    return Image.fromarray(pixels.copy(), mode="RGB")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default="/home/ubuntu/model/experiments/fm9g4b_grounding/data")
    parser.add_argument("--run", required=True, help="New collection folder, e.g. run_001")
    parser.add_argument("--scene-id", help="Use the same ID for every chunk of the same flight")
    parser.add_argument("--topic", default="/rflysim/sensor1/img_rgb")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--max-frames", type=int, default=180)
    parser.add_argument("--first-frame-timeout", type=float, default=30.0)
    parser.add_argument("--require-mount", default="/home/ubuntu/fm9g-data")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.run):
        parser.error("run may contain only letters, digits, _ and -")
    args.scene_id = args.scene_id or args.run
    if not args.scene_id.strip():
        parser.error("scene-id cannot be empty")
    if any(v <= 0 for v in (args.interval, args.seconds, args.max_frames, args.first_frame_timeout)):
        parser.error("Sampling limits must be positive")
    if not Path(args.require_mount).is_mount():
        parser.error(f"Data disk not mounted: {args.require_mount}")
    root = Path(args.data_root).resolve()
    mount = Path(args.require_mount).resolve()
    if not root.is_relative_to(mount):
        parser.error("data-root must be on the required data mount")
    destination = root / "images" / args.run
    if destination.exists():
        parser.error(f"Use a new --run; refusing to overwrite {destination}")
    import rospy
    from sensor_msgs.msg import Image
    rospy.init_node("fm9g_frame_capture", anonymous=True, disable_signals=True)
    lock = threading.Lock()
    latest = [None, 0, 0.0]

    def receive(message):
        with lock:
            latest[0] = message
            latest[1] += 1
            latest[2] = time.monotonic()

    subscriber = rospy.Subscriber(args.topic, Image, receive, queue_size=1, buff_size=4 * 1024 * 1024)
    print(f"Waiting for {args.topic}; this collector does not start flight.", flush=True)
    deadline = time.monotonic() + args.first_frame_timeout
    manifest = None
    count, previous_serial = 0, 0
    started = next_sample = None
    last_stamp = None
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                message, serial, received = latest
            if message is None:
                if now > deadline:
                    raise RuntimeError("No camera image received. Check simulation and the ROS image topic.")
                time.sleep(0.05)
                continue
            if started is None:
                destination.mkdir(parents=True, exist_ok=False)
                metadata = vars(args).copy()
                metadata.update(started_utc=datetime.now(timezone.utc).isoformat(),
                                encoding=message.encoding, width=message.width, height=message.height)
                (destination / "capture.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
                manifest = (destination / "frames.jsonl").open("w", encoding="utf-8")
                started = next_sample = now
            if now - started >= args.seconds or count >= args.max_frames:
                break
            stamp = (message.header.stamp.secs, message.header.stamp.nsecs)
            if now >= next_sample and serial != previous_serial and now - received <= 1.0:
                if last_stamp is not None and stamp != (0, 0) and stamp < last_stamp:
                    raise RuntimeError("ROS time moved backwards; stop and use a new run/scene after reset")
                if stamp == (0, 0) or stamp != last_stamp:
                    image = decode_rgb(message)
                    name = f"frame_{count:06d}.png"
                    image.save(destination / name)
                    record = dict(image=f"images/{args.run}/{name}", scene_id=args.scene_id,
                                  width=image.width, height=image.height, encoding=message.encoding,
                                  stamp_sec=stamp[0], stamp_nsec=stamp[1],
                                  frame_id=message.header.frame_id)
                    manifest.write(json.dumps(record) + "\n")
                    manifest.flush()
                    count += 1
                    print(f"SAVED {count}: {destination / name} ({message.encoding} -> RGB)", flush=True)
                    last_stamp = stamp
                previous_serial = serial
                next_sample = now + args.interval
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("Capture stopped by user.", flush=True)
    finally:
        subscriber.unregister()
        if manifest:
            manifest.close()
        print(f"CAPTURE_FINISHED frames={count} folder={destination}", flush=True)


if __name__ == "__main__":
    main()
