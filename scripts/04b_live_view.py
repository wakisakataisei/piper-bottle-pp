"""カラーと深度のライブ表示（Jetson に接続したモニタで見る）
使い方: python scripts/04b_live_view.py > /dev/null   … q で終了、s で data/ に保存
注意: カメラは1プロセスしか開けない。他のスクリプトを動かす前にこれを閉じる
"""
import sys
import numpy as np
import cv2
import _bootstrap  # noqa: F401
from piper_pp import camera
from piper_pp.config import data_path

pipe = camera.start()
color = depth_vis = None
print("q キーで終了、s キーで画像を保存", file=sys.stderr)
while True:
    f = pipe.wait_for_frames(100)
    if f is not None:
        c, d = f.get_color_frame(), f.get_depth_frame()
        if c is not None:
            color = camera.for_display(camera.decode_color(c))
        if d is not None:
            z = camera.depth_m(d) * 1000
            vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(z, 0, 1500), alpha=255 / 1500), cv2.COLORMAP_JET)
            vis[z == 0] = 0
            depth_vis = camera.for_display(vis)
            h, w = z.shape
            cz = z[h//2-5:h//2+5, w//2-5:w//2+5]; cv_ = cz[cz > 0]
            txt = f"center {np.median(cv_):.0f} mm" if cv_.size else "center invalid"
            cv2.drawMarker(depth_vis, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(depth_vis, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    if color is not None and depth_vis is not None:
        dv = cv2.resize(depth_vis, (color.shape[1], color.shape[0]))
        cv2.imshow("color | depth (0-1.5m)", np.hstack([color, dv]))
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    if k == ord("s") and color is not None:
        cv2.imwrite(str(data_path("snap_color.jpg")), color)
        cv2.imwrite(str(data_path("snap_depth.png")), depth_vis)
        print("保存しました", file=sys.stderr)
pipe.stop(); cv2.destroyAllWindows()
