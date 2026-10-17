"""Scan-Ordner → Punktwolke (reconstruction.ply, Meter, Roboterbasis).

Aufruf (aus dem Projektordner):
    python scripts/reconstruct.py                          # neuester Run in data/runs
    python scripts/reconstruct.py data/runs/20261016_120000 --show
"""
import argparse
import logging
import sys
from pathlib import Path

import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from modelling import ReconstructionConfig, reconstruct  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", nargs="?", help="Scan-Ordner (Standard: neuester)")
    parser.add_argument("--config", default="config/reconstruction.yaml")
    parser.add_argument("--show", action="store_true", help="Ergebnis anzeigen")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    if args.run_dir:
        run_dir = Path(args.run_dir)
    else:
        runs = sorted(p for p in Path("data/runs").glob("*") if (p / "poses.json").exists())
        if not runs:
            sys.exit("Kein Scan in data/runs gefunden")
        run_dir = runs[-1]
    print(f"Run: {run_dir}")

    cfg = (ReconstructionConfig.from_yaml(args.config)
           if Path(args.config).exists() else ReconstructionConfig())
    pcd = reconstruct(run_dir, cfg)

    out = run_dir / "reconstruction.ply"
    o3d.io.write_point_cloud(str(out), pcd)
    print(f"Gespeichert: {out} ({len(pcd.points)} Punkte)")

    if args.show:
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)  # Roboterbasis
        o3d.visualization.draw_geometries([pcd, axes])


if __name__ == "__main__":
    main()