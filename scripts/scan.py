"""Kompletter Scan: Roboter fährt alle Positionen ab, an jeder wird ein Bild aufgenommen.

Aufruf (aus dem Projektordner):
    python scripts/scan.py
    python scripts/scan.py --no-camera     # nur Roboter abfahren, keine Bilder

Ergebnis: data/runs/<datum_uhrzeit>/
    view_001_color.png, view_001_depth.png, view_001_meta.json, ...
    poses.json   (Roboterpose je Bild)
"""
import argparse
import json
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from robot import Robot, RobotConfig, RobotConnectionError, RobotError  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-config", default="config/robot.yaml")
    parser.add_argument("--camera-config", default="config/camera.yaml")
    parser.add_argument("--out", default="data/runs")
    parser.add_argument("--no-camera", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    robot_cfg = (RobotConfig.from_yaml(args.robot_config)
                 if Path(args.robot_config).exists() else RobotConfig())

    run_dir = Path(args.out) / time.strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    poses = []

    camera = None
    if not args.no_camera:
        from camera import CameraConfig, RealSenseCamera
        cam_cfg = (CameraConfig.from_yaml(args.camera_config)
                   if Path(args.camera_config).exists() else CameraConfig())
        camera = RealSenseCamera(cam_cfg)
        print("Kamera starten …")
        camera.open()

    try:
        with Robot(robot_cfg) as robot:
            start = robot.reset()
            print(f"Scan mit {start.total} Positionen → {run_dir}")

            while True:
                s = robot.next()
                name = f"view_{s.index:03d}"
                print(f"[{s.index}/{s.total}] Position erreicht")

                entry = {"index": s.index, "name": name, "pose_mm_quat": s.pose.to_dict(),
                         "T_base_flange_m": s.pose.as_matrix().tolist()}
                if camera is not None:
                    frame = camera.capture()
                    frame.save(run_dir, name)
                    entry["timestamp"] = frame.timestamp
                poses.append(entry)

                if s.last:
                    break
    except (RobotConnectionError, RobotError) as e:
        print(f"FEHLER: {e}")
    finally:
        if camera is not None:
            camera.close()
        (run_dir / "poses.json").write_text(json.dumps(poses, indent=2), encoding="utf-8")

    print(f"Fertig: {len(poses)} Aufnahmen in {run_dir}")


if __name__ == "__main__":
    main()