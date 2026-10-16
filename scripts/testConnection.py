"""Prüft, ob die TCP-Verbindung zum ABB-ScanServer klappt.

Aufruf (aus dem Projektordner):
    python scripts/checkrobot.py              # verbinden, QUIT senden, BYE erwarten
    python scripts/checkrobot.py --no-quit    # nur verbinden und wieder trennen

Achtung: Der ScanServer nimmt nur EINE Verbindung an und beendet sich danach.
Vor jedem Test das RAPID-Programm neu starten.
"""
import argparse
import socket
import sys
from pathlib import Path

import yaml


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/robot.yaml")
    parser.add_argument("--no-quit", action="store_true", help="nur verbinden, kein QUIT senden")
    args = parser.parse_args()

    cfg = {"host": "10.0.0.20", "port": 5000, "connect_timeout": 5.0}
    if Path(args.config).exists():
        cfg.update(yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))["robot"])

    host, port = cfg["host"], int(cfg["port"])
    print(f"Verbinde mit {host}:{port} …")
    try:
        sock = socket.create_connection((host, port), timeout=cfg["connect_timeout"])
    except socket.timeout:
        sys.exit("FEHLER: Timeout – Roboter nicht erreichbar (IP/Netzwerk/Kabel prüfen)")
    except ConnectionRefusedError:
        sys.exit("FEHLER: Verbindung abgelehnt – läuft ScanServer auf dem Roboter?")
    except OSError as e:
        sys.exit(f"FEHLER: {e}")

    print("OK: verbunden")

    with sock:
        if args.no_quit:
            return
        sock.sendall(b"QUIT")
        try:
            reply = sock.recv(1024).decode("ascii", errors="replace")
        except socket.timeout:
            sys.exit("FEHLER: verbunden, aber keine Antwort auf QUIT")
        print(f"Antwort: {reply!r}")
        print("OK: Kommunikation funktioniert" if reply.startswith("BYE") else "Unerwartete Antwort")


if __name__ == "__main__":
    main()