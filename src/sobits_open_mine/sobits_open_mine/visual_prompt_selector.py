from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Select a YOLOE visual prompt bbox with the mouse."
    )
    parser.add_argument("--image", required=True, help="Reference image path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image_path = Path(args.image).expanduser().resolve()
    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    print("Drag around the target, then press ENTER or SPACE. Press C to cancel.")
    x, y, width, height = cv2.selectROI(
        "YOLOE visual prompt selector",
        image,
        showCrosshair=True,
        fromCenter=False,
    )
    cv2.destroyAllWindows()

    if width <= 0 or height <= 0:
        raise RuntimeError("BBox selection was canceled or empty.")

    bbox = [float(x), float(y), float(x + width), float(y + height)]
    output_path = image_path.with_suffix(".bbox.yaml")
    with output_path.open("w", encoding="utf-8") as file:
        yaml.safe_dump({"prompt_bbox": bbox}, file, sort_keys=False)

    print(f"Saved: {output_path}")
    print(f"prompt_bbox: {bbox}")


if __name__ == "__main__":
    main()
