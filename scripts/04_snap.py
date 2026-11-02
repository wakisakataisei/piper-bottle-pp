"""カラーと深度を1枚ずつ保存（data/snap_color.jpg, data/snap_depth.png）
使い方: python scripts/04_snap.py > /dev/null
"""
import sys
import numpy as np
import cv2
import _bootstrap  # noqa: F401
from piper_pp import camera
from piper_pp.config import data_path

out = sys.stderr
depths, color, _ = camera.grab(n_depth=1, need_color=True)
if color is None or not depths:
    sys.exit(f"取得失敗: color {color is not None}, depth {bool(depths)}")
depth = depths[0] * 1000
cv2.imwrite(str(data_path("snap_color.jpg")), camera.for_display(color))
vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(depth, 0, 1500), alpha=255 / 1500), cv2.COLORMAP_JET)
vis[depth == 0] = 0
cv2.imwrite(str(data_path("snap_depth.png")), camera.for_display(vis))
v = depth[depth > 0]
print(f"保存: data/snap_color.jpg {color.shape[1]}x{color.shape[0]}, data/snap_depth.png {depth.shape[1]}x{depth.shape[0]}", file=out)
print(f"深度: 有効 {v.size/depth.size*100:.0f}%  最小 {v.min():.0f} mm  中央値 {np.median(v):.0f} mm" if v.size else "深度: 有効画素なし", file=out)
