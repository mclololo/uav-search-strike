"""Export same-frame YOLO predictions directly as automatic FM9G training labels."""

import argparse
from collections import Counter
from copy import deepcopy
import json
from pathlib import Path

from grounding_data import check_record


def automatic_label(record):
    if (record.get('annotation_source') != 'yolo_ros'
            or record.get('detection_status') != 'matched'
            or not record.get('objects') or record.get('rejected_predictions')):
        return None
    result = deepcopy(record)
    result.update(review_status='auto_accepted', needs_review=False, human_reviewed=False)
    check_record(result, '.', check_image=False)
    return result


def export(source, root, output):
    source, root, output = Path(source), Path(root), Path(output)
    if output.exists():
        raise ValueError('Output exists; choose a new filename')
    rows, seen, counts, skipped = [], set(), Counter(), 0
    for line in source.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        record = automatic_label(json.loads(line))
        if record is None:
            skipped += 1
            continue
        check_record(record, root)
        if record['image'] in seen:
            raise ValueError('Duplicate image in predictions')
        seen.add(record['image'])
        rows.append(record)
        counts.update(o['class_name'] for o in record['objects'])
    if not rows:
        raise ValueError('No matched YOLO detections to export')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
    return dict(images=len(rows), boxes=sum(counts.values()), classes=dict(counts),
                excluded_images=skipped, label_source='yolo_ros', human_reviewed=False,
                output=str(output))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.input, args.data_root, args.output), indent=2))
