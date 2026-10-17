"""Tiefenbilder + Roboterposen → eine Punktwolke in Roboterbasis-Koordinaten (Meter)."""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import yaml

log = logging.getLogger(__name__)


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    q = np.array([w, x, y, z], dtype=float)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


@dataclass
class ReconstructionConfig:
    T_flange_camera: np.ndarray = field(default_factory=lambda: np.eye(4))
    depth_min: float = 0.2
    depth_max: float = 1.0
    crop_min: list[float] = field(default_factory=lambda: [-1.0, -1.0, -1.0])
    crop_max: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    remove_plane: bool = True
    plane_distance: float = 0.003
    voxel_size: float = 0.002
    refine_icp: bool = False
    icp_max_distance: float = 0.005
    outlier_neighbors: int = 20
    outlier_std_ratio: float = 2.0

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ReconstructionConfig":
        with open(path, encoding="utf-8") as f:
            data = (yaml.safe_load(f) or {}).get("reconstruction", {})
        hand_eye = data.pop("T_flange_camera", None)
        cfg = cls(**data)
        if hand_eye:
            T = np.eye(4)
            T[:3, :3] = quat_to_matrix(*hand_eye["quaternion"])
            T[:3, 3] = hand_eye["translation"]
            cfg.T_flange_camera = T
        return cfg


@dataclass
class View:
    name: str
    color: np.ndarray          # BGR
    depth: np.ndarray          # uint16
    depth_scale: float
    intrinsics: dict
    T_base_flange: np.ndarray  # 4x4, Meter


def load_run(run_dir: str | Path) -> list[View]:
    """Liest poses.json und die zugehörigen Bilder eines Scan-Ordners."""
    run_dir = Path(run_dir)
    entries = json.loads((run_dir / "poses.json").read_text(encoding="utf-8"))
    views = []
    for e in entries:
        name = e["name"]
        depth_path = run_dir / f"{name}_depth.png"
        if not depth_path.exists():
            log.warning("%s: kein Tiefenbild, übersprungen", name)
            continue
        meta = json.loads((run_dir / f"{name}_meta.json").read_text(encoding="utf-8"))
        views.append(View(
            name=name,
            color=cv2.imread(str(run_dir / f"{name}_color.png"), cv2.IMREAD_COLOR),
            depth=cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED),
            depth_scale=meta["depth_scale"],
            intrinsics=meta["intrinsics"],
            T_base_flange=np.array(e["T_base_flange_m"]),
        ))
    if not views:
        raise FileNotFoundError(f"Keine verwendbaren Aufnahmen in {run_dir}")
    return views


def view_to_pointcloud(view: View, depth_min: float, depth_max: float) -> o3d.geometry.PointCloud:
    """Eine Aufnahme → farbige Punktwolke im Kamera-Koordinatensystem."""
    intr = view.intrinsics
    pinhole = o3d.camera.PinholeCameraIntrinsic(
        intr["width"], intr["height"], intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    )
    rgb = cv2.cvtColor(view.color, cv2.COLOR_BGR2RGB)
    depth = view.depth.copy()
    depth_m = depth * view.depth_scale
    depth[(depth_m < depth_min) | (depth_m > depth_max)] = 0

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(np.ascontiguousarray(rgb)),
        o3d.geometry.Image(depth),
        depth_scale=1.0 / view.depth_scale,
        depth_trunc=depth_max,
        convert_rgb_to_intensity=False,
    )
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole)


def reconstruct(run_dir: str | Path, cfg: ReconstructionConfig) -> o3d.geometry.PointCloud:
    views = load_run(run_dir)
    crop = o3d.geometry.AxisAlignedBoundingBox(cfg.crop_min, cfg.crop_max)
    merged = o3d.geometry.PointCloud()

    for view in views:
        pcd = view_to_pointcloud(view, cfg.depth_min, cfg.depth_max)
        pcd.transform(view.T_base_flange @ cfg.T_flange_camera)   # → Roboterbasis
        pcd = pcd.crop(crop).voxel_down_sample(cfg.voxel_size)

        # Tisch pro Ansicht entfernen, damit ICP nur am Bauteil ausrichtet
        if cfg.remove_plane and len(pcd.points) > 100:
            _, inliers = pcd.segment_plane(cfg.plane_distance, 3, 1000)
            pcd = pcd.select_by_index(inliers, invert=True)

        if len(pcd.points) == 0:
            log.warning("%s: keine Punkte im Arbeitsbereich", view.name)
            continue

        if cfg.refine_icp and len(merged.points) > 0:
            pcd.transform(_refine(pcd, merged, cfg, view.name))

        merged += pcd
        merged = merged.voxel_down_sample(cfg.voxel_size)
        log.info("%s: %d Punkte gesamt", view.name, len(merged.points))

    if len(merged.points) == 0:
        raise RuntimeError("Leere Punktwolke – Crop-Bereich und T_flange_camera prüfen")

    merged, _ = merged.remove_statistical_outlier(cfg.outlier_neighbors, cfg.outlier_std_ratio)
    merged.estimate_normals()
    log.info("Fertig: %d Punkte", len(merged.points))
    return merged


def _refine(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
            cfg: ReconstructionConfig, name: str) -> np.ndarray:
    """Kleine ICP-Korrektur einer Ansicht. Große Sprünge werden verworfen."""
    source.estimate_normals()
    target.estimate_normals()
    result = o3d.pipelines.registration.registration_icp(
        source, target, cfg.icp_max_distance, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    T = result.transformation
    shift_mm = np.linalg.norm(T[:3, 3]) * 1000
    angle_deg = np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1)))
    ok = result.fitness > 0.3 and shift_mm < 2 * cfg.icp_max_distance * 1000 and angle_deg < 3
    log.info("%s: ICP fitness %.2f, Korrektur %.1f mm / %.2f° → %s",
             name, result.fitness, shift_mm, angle_deg, "übernommen" if ok else "verworfen")
    return T if ok else np.eye(4)