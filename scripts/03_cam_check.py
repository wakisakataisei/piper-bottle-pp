"""カメラの確認: 内部パラメータ、深度→カラーの外部パラメータ、中央の深度、最短距離
使い方: python scripts/03_cam_check.py > /dev/null   （SDK のログが標準出力に出るため結果は標準エラーに出す）
確認すること: fx>0、深度が取れる。手のひらをカメラに近づけ、何 mm で無効になるかで最短距離を確かめる
"""
import sys
import time
import numpy as np
import _bootstrap  # noqa: F401
from piper_pp import camera

out = sys.stderr
pipe = camera.start()
p = pipe.get_camera_param()
di, ri = p.depth_intrinsic, p.rgb_intrinsic
R, t = camera.depth_to_color_extrinsics(p)
print(f"カラー: fx={ri.fx:.1f} fy={ri.fy:.1f} cx={ri.cx:.1f} cy={ri.cy:.1f} ({ri.width}x{ri.height})", file=out)
print(f"深度  : fx={di.fx:.1f} fy={di.fy:.1f} cx={di.cx:.1f} cy={di.cy:.1f} ({di.width}x{di.height})", file=out)
print(f"深度→カラー 平行移動: {np.round(t*1000, 1)} mm", file=out)
print("中央 10x10 画素の深度を 10 秒表示します（手のひらを近づけて最短距離を確認）", file=out)
t0, nmin = time.time(), None
while time.time() - t0 < 10:
    fr = pipe.wait_for_frames(200)
    d = fr.get_depth_frame() if fr else None
    if d is None:
        continue
    z = camera.depth_m(d) * 1000
    h, w = z.shape
    c = z[h//2-5:h//2+5, w//2-5:w//2+5]; v = c[c > 0]
    if v.size:
        m = float(np.median(v)); nmin = m if nmin is None else min(nmin, m)
        print(f"\r中央 {m:6.0f} mm  有効 {v.size}/100  これまでの最小 {nmin:.0f} mm   ", end="", flush=True, file=out)
    else:
        print("\r中央 無効（近すぎる/遠すぎる）                         ", end="", flush=True, file=out)
pipe.stop()
print("\n完了", file=out)
