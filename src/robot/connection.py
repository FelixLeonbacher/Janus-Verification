"""WebSocket-Client für die Kommunikation mit dem Roboter (Roboter = Server)."""
from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from websockets.exceptions import ConnectionClosed, InvalidHandshake, InvalidURI
from websockets.sync.client import ClientConnection, connect

log = logging.getLogger(__name__)


class RobotConnectionError(RuntimeError):
    """Verbindung fehlgeschlagen, getrennt oder Timeout."""


@dataclass
class RobotConfig:
    url: str = "ws://192.168.0.10:8080"
    connect_timeout: float = 5.0
    read_timeout: float = 10.0
    connect_retries: int = 3
    retry_delay: float = 2.0
    ping_interval: float | None = 20.0
    ping_timeout: float | None = 20.0

    @classmethod
    def from_yaml(cls, path: str | Path) -> "RobotConfig":
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(**data.get("robot", {}))


class RobotConnection:
    def __init__(self, config: RobotConfig | None = None):
        self.config = config or RobotConfig()
        self._ws: ClientConnection | None = None

    # ------------------------------------------------------------------ Verbindung

    @property
    def is_connected(self) -> bool:
        return self._ws is not None

    def connect(self) -> None:
        if self.is_connected:
            return
        cfg = self.config
        last_error: Exception | None = None

        for attempt in range(1, cfg.connect_retries + 1):
            try:
                log.info("Verbinde mit %s (Versuch %d/%d)", cfg.url, attempt, cfg.connect_retries)
                self._ws = connect(
                    cfg.url,
                    open_timeout=cfg.connect_timeout,
                    ping_interval=cfg.ping_interval,
                    ping_timeout=cfg.ping_timeout,
                )
                log.info("Verbunden mit %s", cfg.url)
                return
            except InvalidURI as e:
                raise RobotConnectionError(f"Ungültige URL: {cfg.url}") from e
            except (OSError, TimeoutError, InvalidHandshake) as e:
                last_error = e
                log.warning("Verbindung fehlgeschlagen: %s", e)
                if attempt < cfg.connect_retries:
                    time.sleep(cfg.retry_delay)

        raise RobotConnectionError(
            f"Keine Verbindung zu {cfg.url} nach {cfg.connect_retries} Versuchen: {last_error}"
        )

    def close(self) -> None:
        if self._ws is not None:
            try:
                self._ws.close()
            finally:
                self._ws = None
                log.info("Verbindung geschlossen")

    def reconnect(self) -> None:
        self.close()
        self.connect()

    # ------------------------------------------------------------------ Senden

    def send(self, message: str | bytes) -> None:
        ws = self._require_connection()
        try:
            ws.send(message)
            log.debug("→ %s", message)
        except ConnectionClosed as e:
            self._ws = None
            raise RobotConnectionError(f"Verbindung beim Senden getrennt: {e}") from e

    def send_json(self, data: dict[str, Any]) -> None:
        self.send(json.dumps(data))

    # ------------------------------------------------------------------ Lesen

    def read(self, timeout: float | None = None) -> str | bytes:
        """Wartet auf die nächste Nachricht. timeout=None → read_timeout aus der Config."""
        ws = self._require_connection()
        timeout = self.config.read_timeout if timeout is None else timeout
        try:
            message = ws.recv(timeout=timeout)
        except TimeoutError as e:
            raise RobotConnectionError(f"Keine Antwort innerhalb von {timeout} s") from e
        except ConnectionClosed as e:
            self._ws = None
            raise RobotConnectionError(f"Verbindung beim Lesen getrennt: {e}") from e
        log.debug("← %s", message)
        return message

    def read_json(self, timeout: float | None = None) -> dict[str, Any]:
        message = self.read(timeout)
        try:
            return json.loads(message)
        except json.JSONDecodeError as e:
            raise RobotConnectionError(f"Antwort ist kein JSON: {message!r}") from e

    def request(self, data: dict[str, Any], timeout: float | None = None) -> dict[str, Any]:
        """Sendet JSON und wartet auf die nächste JSON-Antwort."""
        self.send_json(data)
        return self.read_json(timeout)

    # ------------------------------------------------------------------ Intern

    def _require_connection(self) -> ClientConnection:
        if self._ws is None:
            raise RobotConnectionError("Nicht verbunden – erst connect() aufrufen")
        return self._ws

    def __enter__(self) -> "RobotConnection":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()