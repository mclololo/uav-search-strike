"""Strict pixel-coordinate annotations for the FM9GV grounding experiment."""

import hashlib
import json
import math
from pathlib import Path


CLASSES = ("square_frame", "red_balloon", "blue_balloon", "purple_balloon")
PROMPT = (
    "Locate every visible square frame and red, blue or purple balloon in this image. "
    "Return only JSON with this schema: "
    '{"objects":[{"class_name":"red_balloon","bbox":[left,top,right,bottom]}]}. '
    "Allowed class_name values: square_frame, red_balloon, blue_balloon, purple_balloon. "
    "Coordinates are integers from 0 to 1000 relative to the entire input image. "
    'If none are visible, return {"objects":[]}. Do not guess hidden objects.'
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def check_record(record, root, check_image=True):
    if not isinstance(record, dict):
        raise ValueError("Annotation must be an object")
    auto = record.get("review_status") == "auto_accepted"
    if (record.get("needs_review") is True
            or record.get("review_status", "reviewed") not in ("reviewed", "auto_accepted")
            or (record.get("annotation_source") == "yolo_ros"
                and record.get("review_status") not in ("reviewed", "auto_accepted"))):
        raise ValueError("Unreviewed predictions cannot be used as verified training labels")
    if auto and (record.get("annotation_source") != "yolo_ros"
                 or record.get("detection_status") != "matched"
                 or not record.get("objects") or record.get("rejected_predictions")
                 or record.get("human_reviewed") is not False):
        raise ValueError("Automatic labels require matched, valid YOLO boxes and explicit provenance")
    width, height = record.get("width"), record.get("height")
    if any(type(v) is not int or v <= 0 for v in (width, height)):
        raise ValueError("width and height must be positive integers")
    if not isinstance(record.get("scene_id"), str) or not record["scene_id"].strip():
        raise ValueError("A nonempty scene_id is required for split isolation")
    if not isinstance(record.get("image"), str) or not record["image"]:
        raise ValueError("image must be a relative path")
    root = Path(root).resolve()
    image_path = (root / record["image"]).resolve()
    if Path(record["image"]).is_absolute() or not image_path.is_relative_to(root):
        raise ValueError("image must stay within the dataset root")
    objects = record.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects must be a list, including [] for an empty image")
    for obj in objects:
        if not isinstance(obj, dict) or obj.get("class_name") not in CLASSES:
            raise ValueError("Unknown class_name")
        if "visible" in obj and obj["visible"] is not True:
            raise ValueError("Do not annotate fully invisible objects")
        bbox = obj.get("bbox")
        if not isinstance(bbox, list) or len(bbox) != 4:
            raise ValueError("bbox must contain four pixel coordinates")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in bbox):
            raise ValueError("bbox coordinates must be finite numbers")
        x1, y1, x2, y2 = bbox
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError("bbox must be ordered and inside the original image")
        norm = [round(x1 * 1000 / width), round(y1 * 1000 / height),
                round(x2 * 1000 / width), round(y2 * 1000 / height)]
        if norm[0] >= norm[2] or norm[1] >= norm[3]:
            raise ValueError("bbox is too small for the 0..1000 output grid")
    if check_image:
        from PIL import Image
        with Image.open(image_path) as image:
            if image.size != (width, height):
                raise ValueError("Annotation dimensions differ from the image")
            image.verify()
    return record


def load_records(path, root, require_reviewed=False):
    records = []
    with Path(path).open(encoding="utf-8") as stream:
        for line_num, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = check_record(json.loads(line), root)
                if require_reviewed and record.get("review_status", "reviewed") != "reviewed":
                    raise ValueError("Validation/test labels must be reviewed independently of YOLO")
            except (ValueError, OSError) as error:
                raise ValueError(f"{path}:{line_num}: {error}") from error
            records.append(record)
    if not records:
        raise ValueError(f"No annotated images in {path}")
    return records


def check_splits(train, val, root):
    overlap = {r["scene_id"] for r in train} & {r["scene_id"] for r in val}
    if overlap:
        raise ValueError(f"Train/validation share scenes: {sorted(overlap)}")
    train_hashes = {sha256(Path(root) / r["image"]) for r in train}
    if any(sha256(Path(root) / r["image"]) in train_hashes for r in val):
        raise ValueError("Identical image bytes occur in both splits")


def answer_for(record):
    width, height = record["width"], record["height"]
    objects = []
    for obj in sorted(record["objects"], key=lambda o: (o["class_name"], *o["bbox"])):
        x1, y1, x2, y2 = obj["bbox"]
        objects.append({"class_name": obj["class_name"], "bbox": [
            round(x1 * 1000 / width), round(y1 * 1000 / height),
            round(x2 * 1000 / width), round(y2 * 1000 / height)]})
    return json.dumps({"objects": objects}, separators=(",", ":"))
