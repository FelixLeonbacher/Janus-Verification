"""Ein Foto mit der RealSense D435 aufnehmen und speichern.

Aufruf (aus dem Projektordner):
    python scripts/take_photo.py
    python scripts/take_photo.py --out data/test --show
    python scripts/take_photo.py --no-depth
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from camera import CameraConfig, RealSenseCamera  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/camera.yaml")
    parser.add_argument("--out", default="data/test_captures")
    parser.add_argument("--no-depth", action="store_true", help="nur Farbbild")
    parser.add_argument("--show", action="store_true", help="Bild anzeigen")
    args = parser.parse_args()

    cfg = CameraConfig.from_yaml(args.config) if Path(args.config).exists() else CameraConfig()
    if args.no_depth:
        cfg.depth = False

    with RealSenseCamera(cfg) as cam:
        frame = cam.capture()

    name = time.strftime("%Y%m%d_%H%M%S")
    for kind, path in frame.save(args.out, name).items():
        print(f"{kind:6s} → {path}")

    if args.show:
        cv2.imshow("color", frame.color)
        if frame.depth is not None:
            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(frame.depth, alpha=0.03), cv2.COLORMAP_JET
            )
            cv2.imshow("depth", depth_vis)
        print("Taste drücken zum Schließen …")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()