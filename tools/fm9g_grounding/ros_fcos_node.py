"""ROS1 bridge for the FM9G spatial detector; contains no YOLO dependency."""

import argparse
import time

from capture_frames import decode_rgb


def image_to_rgb(message):
    import numpy as np
    return np.asarray(decode_rgb(message))


def selected_boxes(detected, width, height, classes, threshold, max_detections):
    boxes = []
    for coords, label, score in zip(detected["boxes"].tolist(),
                                    detected["labels"].tolist(),
                                    detected["scores"].tolist()):
        if score < threshold:
            continue
        if len(boxes) >= max_detections:
            break
        if not 0 <= label < len(classes):
            raise ValueError(f"Unknown detector label {label}")
        x1, y1, x2, y2 = coords
        x1, x2 = max(0, min(width, round(x1))), max(0, min(width, round(x2)))
        y1, y2 = max(0, min(height, round(y1))), max(0, min(height, round(y2)))
        if x1 < x2 and y1 < y2:
            boxes.append((classes[label], (x1, y1, x2, y2), score))
    return boxes


def main():
    import cv2
    import rospy
    import torch
    from common_msgs.msg import DetEnable, Obj, Objects
    from sensor_msgs.msg import Image
    from fcos_grounding import build_detector, load_head
    from grounding_data import CLASSES

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="/home/ubuntu/model/FM9G4B-V")
    parser.add_argument("--head", required=True)
    parser.add_argument("--image-size", type=int, default=448)
    parser.add_argument("--score-threshold", type=float, default=0.43)
    parser.add_argument("--max-detections", type=int, default=8)
    parser.add_argument("--image-topic", default="/rflysim/sensor1/img_rgb")
    parser.add_argument("--objects-topic", default="/objects")
    parser.add_argument("--control-topic", default="/detect_ctrl")
    parser.add_argument("--show-window", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    if not 0.01 <= args.score_threshold < 1 or args.max_detections < 1:
        parser.error("Invalid score threshold or max detections")

    torch.set_num_threads(4)
    torch.set_grad_enabled(False)
    rospy.init_node("fm9g_spatial_detector")
    master = rospy.get_master()
    _, _, (publishers, _, _) = master.getSystemState()
    current = next((nodes for topic, nodes in publishers
                    if topic == args.objects_topic), [])
    if current:
        raise RuntimeError(f"{args.objects_topic} already has publishers: {current}")

    model = build_detector(args.base, args.image_size)
    load_head(model, args.head)
    model.eval()
    output = rospy.Publisher(args.objects_topic, Objects, queue_size=1)
    enabled = True
    window = "FM9G Detection"

    def control_callback(message):
        nonlocal enabled
        enabled = message.is_open

    def image_callback(message):
        if not enabled:
            return
        start = time.perf_counter()
        try:
            rgb = image_to_rgb(message)
            tensor = torch.from_numpy(rgb.copy()).permute(2, 0, 1).float().div_(255).cuda()
            with torch.inference_mode():
                detected = model([tensor])[0]
            boxes = selected_boxes(detected, message.width, message.height,
                                   CLASSES, args.score_threshold, args.max_detections)
        except Exception as error:
            rospy.logerr_throttle(5, "FM9G detection failed: %s", error)
            return
        result = Objects()
        result.header = message.header
        result.source_id = 1
        for name, (x1, y1, x2, y2), score in boxes:
            obj = Obj()
            obj.header = message.header
            obj.class_name = name
            obj.left_top_x, obj.left_top_y = x1, y1
            obj.right_bottom_x, obj.right_bottom_y = x2, y2
            obj.center_x, obj.center_y = (x1 + x2) // 2, (y1 + y2) // 2
            obj.score = score
            result.objects.append(obj)
        output.publish(result)
        elapsed = time.perf_counter() - start
        rospy.loginfo_throttle(5, "FM9G detection %.1f ms, %d objects", elapsed * 1000, len(boxes))
        if args.show_window:
            display = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for name, (x1, y1, x2, y2), score in boxes:
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 220, 255), 2)
                cv2.putText(display, f"{name} {score:.2f}", (x1, max(18, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
            cv2.imshow(window, display)
            cv2.waitKey(1)

    rospy.Subscriber(args.control_topic, DetEnable, control_callback, queue_size=1)
    rospy.Subscriber(args.image_topic, Image, image_callback, queue_size=1,
                     buff_size=2 ** 22)
    rospy.loginfo("FM9G detector ready: %s -> %s, head=%s", args.image_topic,
                  args.objects_topic, args.head)
    rospy.spin()
    if args.show_window:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
