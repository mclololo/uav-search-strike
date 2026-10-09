"""Exercise the CLI on synthetic empty images, never a task-quality benchmark."""

import json
from pathlib import Path
import subprocess
import sys
import time

from PIL import Image


def main():
    experiment = Path(__file__).resolve().parent.parent
    root = experiment / "eval" / ("pipeline-SMOKE-ONLY-" + time.strftime("%Y%m%d-%H%M%S"))
    (root / "images").mkdir(parents=True, exist_ok=False)
    records = []
    for index, value in enumerate([240, 245, 250, 255]):
        name = f"images/synthetic-empty-{index}.png"
        Image.new("RGB", (640, 480), (value, value, value)).save(root / name)
        records.append(dict(image=name, width=640, height=480,
                            scene_id="smoke_train" if index < 3 else "smoke_val",
                            objects=[]))
    for name, split in [("train.jsonl", records[:3]), ("val.jsonl", records[3:])]:
        (root / name).write_text("".join(json.dumps(r) + "\n" for r in split), encoding="utf-8")
    args = ["--data-root", str(root), "--train", str(root / "train.jsonl"),
            "--val", str(root / "val.jsonl")]
    cli = str(Path(__file__).with_name("fm9g_grounding.py"))
    subprocess.run([sys.executable, cli, "validate", *args], check=True)
    subprocess.run([sys.executable, cli, "train", *args,
                    "--output", str(root / "adapter-SMOKE-ONLY"),
                    "--epochs", "1", "--accumulation", "2"], check=True)
    result = dict(smoke_only=True, train_images=3, val_images=1,
                  note="Synthetic empty images only. Validation loss is not task accuracy.",
                  output=str(root))
    (root / "pipeline-result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"event": "PIPELINE_SMOKE_OK", **result}), flush=True)


if __name__ == "__main__":
    main()
