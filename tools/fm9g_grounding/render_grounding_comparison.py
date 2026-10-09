"""Render fixed validation examples; this does not choose or tune a model."""

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines()
            if line.strip()]


def render(image_root, base_path, adapter_path, output, indices):
    base = {r["image"]: r for r in read_rows(base_path)}
    rows = read_rows(adapter_path)
    colors = {"square_frame": "#12af94", "red_balloon": "#ff3434",
              "blue_balloon": "#2f78ff", "purple_balloon": "#e13bea"}
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 18)
        small = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 15)
    except OSError:
        font = small = ImageFont.load_default()
    panel_w, panel_h, gap, row_h = 480, 360, 16, 434
    sheet = Image.new("RGB", (3 * panel_w + 4 * gap, len(indices) * row_h + 58), "#f0f3f6")
    draw = ImageDraw.Draw(sheet)
    draw.text((gap, 12), "Validation examples: reviewed labels / base model / LoRA epoch-02", fill="#192533", font=font)
    root = Path(image_root).resolve()
    for display_index, index in enumerate(indices):
        row = rows[index]
        relative = Path(row["image"])
        path = (root / Path(*relative.parts[1:])).resolve()
        if relative.parts[0] != "images" or not path.is_relative_to(root):
            raise ValueError("Image path outside image root")
        with Image.open(path) as source:
            original = source.convert("RGB").resize((panel_w, panel_h))
        panels = [("REVIEWED LABELS", row["ground_truth"]),
                  ("BASE MODEL", base[row["image"]]["prediction"]),
                  ("LoRA EPOCH-02", row["prediction"])]
        y = 50 + display_index * row_h
        draw.text((gap, y), row["image"], fill="#192533", font=small)
        for column, (title, objects) in enumerate(panels):
            x = gap + column * (panel_w + gap)
            draw.text((x, y + 22), title, fill="#192533", font=font)
            panel = original.copy()
            pd = ImageDraw.Draw(panel)
            for obj in objects:
                color = colors[obj["class_name"]]
                x1, y1, x2, y2 = obj["bbox"]
                bbox = (x1 * panel_w / 1000, y1 * panel_h / 1000,
                        min(panel_w - 1, x2 * panel_w / 1000), min(panel_h - 1, y2 * panel_h / 1000))
                pd.rectangle(bbox, outline=color, width=3)
                label = obj["class_name"]
                tx = min(bbox[0], panel_w - pd.textlength(label, font=small) - 4)
                ty = max(0, bbox[1] - 19)
                pd.rectangle((tx, ty, tx + pd.textlength(label, font=small) + 4, ty + 19), fill="#14202b")
                pd.text((tx + 2, ty), label, font=small, fill=color)
            sheet.paste(panel, (x, y + 48))
    sheet.save(output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-root", required=True)
    parser.add_argument("--base-predictions", required=True)
    parser.add_argument("--adapter-predictions", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--indices", type=int, nargs="+", default=[0, 16, 37, 83])
    args = parser.parse_args()
    render(args.image_root, args.base_predictions, args.adapter_predictions,
           args.output, args.indices)
