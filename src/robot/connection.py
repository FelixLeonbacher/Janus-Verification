from __future__ import annotations

import logging
import socket
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

log = logging.getLogger(__name__)


class RobotConnectionError(RuntimeError):
    """Verbindung fehlgeschlagen, getrennt oder Timeout."""


@dataclass
class RobotConfig:
    host: str = "10.0.0.20"
    port: int = 5000
    connect_timeout: float = 5.0
    read_timeout: float = 5.0
    move_timeout: float = 120.0

    @classmethod
    def from_yaml(cls, path: str | Path) -> "RobotConfig":
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(**data.get("robot", {}))


class RobotConnection:
    def __init__(self, config: RobotConfig | None = None):
        self.config = config or RobotConfig()
        self._sock: socket.socket | None = None
        self._buffer = b""

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        if self.is_connected:
            return
        cfg = self.config
        log.info("Verbinde mit %s:%d", cfg.host, cfg.port)
        try:
            self._sock = socket.create_connection((cfg.host, cfg.port), timeout=cfg.connect_timeout)
        except socket.timeout as e:
            raise RobotConnectionError(
                f"Timeout – {cfg.host}:{cfg.port} nicht erreichbar (IP/Netzwerk prüfen)"
            ) from e
        except ConnectionRefusedError as e:
            raise RobotConnectionError(
                f"Verbindung abgelehnt – läuft ScanFixedMain auf dem Roboter?"
            ) from e
        except OSError as e:
            raise RobotConnectionError(f"Verbindung fehlgeschlagen: {e}") from e
        self._buffer = b""
        log.info("Verbunden")

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
                self._buffer = b""
                log.info("Verbindung geschlossen")

    def send(self, command: str) -> None:
        """Sendet ein Kommando (ohne Zeilenende, wie vom Protokoll verlangt)."""
        sock = self._require()
        try:
            sock.sendall(command.encode("ascii"))
        except OSError as e:
            self.close()
            raise RobotConnectionError(f"Senden fehlgeschlagen: {e}") from e
        log.debug("→ %s", command)

    def read_line(self, timeout: float | None = None) -> str:
        """Liest bis zum nächsten LF und gibt die Zeile ohne LF zurück."""
        sock = self._require()
        timeout = self.config.read_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout

        while b"\n" not in self._buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RobotConnectionError(f"Keine Antwort innerhalb von {timeout} s")
            sock.settimeout(remaining)
            try:
                chunk = sock.recv(1024)
            except socket.timeout:
                continue
            except OSError as e:
                self.close()
                raise RobotConnectionError(f"Lesen fehlgeschlagen: {e}") from e
            if not chunk:
                self.close()
                raise RobotConnectionError("Verbindung vom Roboter geschlossen")
            self._buffer += chunk

        line, self._buffer = self._buffer.split(b"\n", 1)
        text = line.decode("ascii", errors="replace").strip()
        log.debug("← %s", text)
        return text

    def request(self, command: str, timeout: float | None = None) -> str:
        """Ein Kommando senden und auf genau eine Antwortzeile warten."""
        self.send(command)
        return self.read_line(timeout)

    def _require(self) -> socket.socket:
        if self._sock is None:
            raise RobotConnectionError("Nicht verbunden – erst connect() aufrufen")
        return self._sock

    def __enter__(self) -> "RobotConnection":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()