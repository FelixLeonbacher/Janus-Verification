"""Gespeichertes 16-bit-Tiefenbild prüfen und farbig anzeigen.

Aufruf: python scripts/show_depth.py data/test_captures/<name>_depth.png
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

path = Path(sys.argv[1])
depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)          # uint16, NICHT nach 8 bit wandeln

meta_path = path.with_name(path.name.replace("_depth.png", "_meta.json"))
scale = json.loads(meta_path.read_text())["depth_scale"] if meta_path.exists() else 0.001

meters = depth.astype(np.float32) * scale
valid = meters > 0
print(f"dtype: {depth.dtype}, shape: {depth.shape}")
print(f"gültige Pixel: {valid.mean() * 100:.1f} %")
if valid.any():
    v = meters[valid]
    print(f"Tiefe [m]: min {v.min():.3f} | median {np.median(v):.3f} | max {v.max():.3f}")

# auf den tatsächlichen Bereich normieren (1.–99. Perzentil), ungültig = schwarz
lo, hi = np.percentile(meters[valid], [1, 99]) if valid.any() else (0, 1)
norm = np.clip((meters - lo) / max(hi - lo, 1e-6), 0, 1)
vis = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
vis[~valid] = 0

cv2.imshow(f"depth {lo:.2f} m – {hi:.2f} m", vis)
cv2.waitKey(0)
cv2.destroyAllWindows()