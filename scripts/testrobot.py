"""Verbindung zum Roboter prüfen: verbinden, Pose abfragen, sauber trennen.

Aufruf (aus dem Projektordner):
    python scripts/checkrobot.py
"""
import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from robot import Robot, RobotConfig, RobotConnectionError, RobotError  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/robot.yaml")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    cfg = RobotConfig.from_yaml(args.config) if Path(args.config).exists() else RobotConfig()

    try:
        with Robot(cfg) as robot:
            s = robot.pos()
            p = s.pose
            print(f"OK: verbunden – Position {s.index}/{s.total}")
            print(f"Pose: x={p.x:.2f} y={p.y:.2f} z={p.z:.2f} mm | "
                  f"q=({p.qw:.4f}, {p.qx:.4f}, {p.qy:.4f}, {p.qz:.4f})")
    except (RobotConnectionError, RobotError) as e:
        sys.exit(f"FEHLER: {e}")


if __name__ == "__main__":
    main()