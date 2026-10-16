"""Verbindung zum Roboter testen: verbinden, Nachricht senden, Antwort lesen.

Aufruf (aus dem Projektordner):
    python scripts/testrobot.py
    python scripts/testrobot.py --msg '{"cmd": "status"}'
    python scripts/testrobot.py --interactive
"""
import argparse
import json
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from robot import RobotConfig, RobotConnection, RobotConnectionError  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/robot.yaml")
    parser.add_argument("--msg", default='{"cmd": "ping"}', help="zu sendende Nachricht")
    parser.add_argument("--interactive", action="store_true", help="Nachrichten eintippen")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    cfg = RobotConfig.from_yaml(args.config) if Path(args.config).exists() else RobotConfig()

    try:
        with RobotConnection(cfg) as robot:
            if not args.interactive:
                robot.send(args.msg)
                print("Antwort:", robot.read())
                return

            print("Nachricht eingeben (leer = Ende):")
            while msg := input("> ").strip():
                robot.send(msg)
                try:
                    print("Antwort:", robot.read())
                except RobotConnectionError as e:
                    print("Fehler:", e)
    except RobotConnectionError as e:
        sys.exit(f"Fehler: {e}")


if __name__ == "__main__":
    main()