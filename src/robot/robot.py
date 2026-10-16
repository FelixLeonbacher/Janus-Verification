from __future__ import annotations

import re
from dataclasses import asdict, dataclass

import numpy as np

from .connection import RobotConfig, RobotConnection, RobotConnectionError


class RobotError(RuntimeError):
    """Roboter hat mit ERR geantwortet."""


@dataclass
class Pose:
    """TCP tool0 in wobj0. Position in mm, Quaternion (w, x, y, z) nach ABB-Konvention."""

    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float

    def as_matrix(self) -> np.ndarray:
        """4x4 homogene Transformation Basis → Flansch, Translation in METERN."""
        q = np.array([self.qw, self.qx, self.qy, self.qz], dtype=float)
        w, x, y, z = q / np.linalg.norm(q)
        T = np.eye(4)
        T[:3, :3] = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
        T[:3, 3] = np.array([self.x, self.y, self.z]) / 1000.0
        return T

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Status:
    index: int      # aktuelle Position (0 nach RESET)
    total: int      # Anzahl Positionen N
    pose: Pose
    last: bool      # True bei LAST → Scan fertig


_RESPONSE = re.compile(
    r"^(OK|LAST)\s+(\d+)\s*/\s*(\d+)\s*\[([^\]]*)\]\s*\[([^\]]*)\]\s*$"
)


def parse_response(line: str) -> Status:
    if line.startswith("ERR"):
        raise RobotError(line[3:].strip() or "unbekannter Fehler")
    m = _RESPONSE.match(line)
    if not m:
        raise RobotError(f"Unerwartete Antwort: {line!r}")
    kind, i, n, trans, rot = m.groups()
    t = [float(v) for v in trans.split(",")]
    r = [float(v) for v in rot.split(",")]
    if len(t) != 3 or len(r) != 4:
        raise RobotError(f"Pose nicht lesbar: {line!r}")
    return Status(int(i), int(n), Pose(*t, *r), kind == "LAST")


class Robot:
    def __init__(self, config: RobotConfig | None = None):
        self.config = config or RobotConfig()
        self.connection = RobotConnection(self.config)

    def connect(self) -> None:
        self.connection.connect()

    def close(self) -> None:
        """Sauber beenden: QUIT senden (falls verbunden), dann Socket schließen."""
        if self.connection.is_connected:
            try:
                self.quit()
            except (RobotConnectionError, RobotError):
                pass
        self.connection.close()

    def next(self) -> Status:
        """Nächste Position anfahren. Kehrt zurück, wenn der Roboter steht."""
        return parse_response(self.connection.request("NEXT", self.config.move_timeout))

    def reset(self) -> Status:
        """Zähler auf Anfang setzen (keine Fahrt)."""
        return parse_response(self.connection.request("RESET"))

    def pos(self) -> Status:
        """Aktuelle Pose abfragen (keine Fahrt)."""
        return parse_response(self.connection.request("POS"))

    def quit(self) -> None:
        reply = self.connection.request("QUIT")
        self.connection.close()
        if not reply.startswith("BYE"):
            raise RobotError(f"Unerwartete Antwort auf QUIT: {reply!r}")

    def __enter__(self) -> "Robot":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()