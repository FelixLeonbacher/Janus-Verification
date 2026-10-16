from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml


@dataclass
class CameraConfig:
    width: int = 1280
    height: int = 720
    fps: int = 30
    depth: bool = True
    warmup_frames: int = 30
    serial: str | None = None

    @classmethod
    def from_yaml(cls, path: str | Path) -> "CameraConfig":
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls(**data.get("camera", {}))


@dataclass
class Frame:
    color: np.ndarray            # BGR, uint8, HxWx3
    depth: np.ndarray | None     # uint16, HxW, auf Farbbild ausgerichtet
    depth_scale: float           # Rohwert * depth_scale = Meter
    intrinsics: dict             # Intrinsics des Farbbilds (Werkskalibrierung)
    timestamp: float

    def save(self, directory: str | Path, name: str) -> dict[str, Path]:
        """Speichert <name>_color.png, <name>_depth.png (16 bit) und <name>_meta.json."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        paths: dict[str, Path] = {}

        paths["color"] = directory / f"{name}_color.png"
        cv2.imwrite(str(paths["color"]), self.color)

        if self.depth is not None:
            paths["depth"] = directory / f"{name}_depth.png"
            cv2.imwrite(str(paths["depth"]), self.depth)

        paths["meta"] = directory / f"{name}_meta.json"
        meta = {
            "timestamp": self.timestamp,
            "depth_scale": self.depth_scale,
            "intrinsics": self.intrinsics,
        }
        paths["meta"].write_text(json.dumps(meta, indent=2), encoding="utf-8")
        return paths


class RealSenseCamera:
    def __init__(self, config: CameraConfig | None = None):
        self.config = config or CameraConfig()
        self._pipeline: rs.pipeline | None = None
        self._align: rs.align | None = None
        self._depth_scale = 0.0
        self._intrinsics: dict = {}

    def open(self) -> None:
        cfg = self.config
        rs_cfg = rs.config()
        if cfg.serial:
            rs_cfg.enable_device(cfg.serial)
        rs_cfg.enable_stream(rs.stream.color, cfg.width, cfg.height, rs.format.bgr8, cfg.fps)
        if cfg.depth:
            rs_cfg.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, cfg.fps)

        pipeline = rs.pipeline()
        try:
            profile = pipeline.start(rs_cfg)
        except RuntimeError as e:
            raise RuntimeError(
                f"RealSense konnte nicht gestartet werden ({e}). "
                "Kamera an USB 3 angeschlossen? Auflösung/FPS unterstützt?"
            ) from e
        self._pipeline = pipeline

        if cfg.depth:
            self._depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self._align = rs.align(rs.stream.color)

        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self._intrinsics = {
            "width": intr.width,
            "height": intr.height,
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.ppx,
            "cy": intr.ppy,
            "model": str(intr.model),
            "coeffs": list(intr.coeffs),
        }

        for _ in range(cfg.warmup_frames):
            pipeline.wait_for_frames()

    def close(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None

    def capture(self, timeout_ms: int = 5000) -> Frame:
        if self._pipeline is None:
            raise RuntimeError("Kamera nicht geöffnet – erst open() aufrufen")

        frames = self._pipeline.wait_for_frames(timeout_ms)
        if self._align is not None:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("Kein Farbbild erhalten")
        color = np.asanyarray(color_frame.get_data()).copy()

        depth = None
        if self.config.depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth = np.asanyarray(depth_frame.get_data()).copy()

        return Frame(color, depth, self._depth_scale, dict(self._intrinsics), time.time())

    def __enter__(self) -> "RealSenseCamera":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()