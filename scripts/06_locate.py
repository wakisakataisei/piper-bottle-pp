"""look 姿勢からボトルのキャップ位置をロボット座標で求める（腕は動かさない）
使い方: python scripts/06_locate.py [--surface 0.021] [--center 0.35 0.0] [--radius 0.30] > /dev/null
  腕は look 姿勢で静止させておく（02_jog.py で go look）。
  確認画像 data/locate_result.jpg（緑=ボトルと判定した点、赤十字=キャップ上端）
確認すること: 高さがボトル実測値と ±10 mm、キャップ位置がメジャー実測と ±5 mm 程度で一致
"""
import sys
import numpy as np
import cv2
import _bootstrap  # noqa: F401
from piper_pp.config import CFG, data_path
from piper_pp.robot import connect
from piper_pp import vision


def opt(name, n=1, default=None):
    if name not in sys.argv:
        return default
    i = sys.argv.index(name)
    v = [float(a) for a in sys.argv[i + 1:i + 1 + n]]
    return v[0] if n == 1 else v


out = sys.stderr
surface = opt("--surface", default=CFG["surface_z"])
center = opt("--center", 2, [0.35, 0.0])
radius = opt("--radius", default=0.30)

robot = connect()
P, T, color, param = vision.points_in_base(robot, need_color=True)
robot.disconnect()
r = vision.locate_in(P, T, center, radius, surface)
if r is None:
    sys.exit(f"置き面 z={surface*1000:.0f} mm より上にボトルが見つかりません（look 姿勢・--surface を確認）")
cap = r["cap"]
print("\n=== 結果（ロボット座標）===", file=out)
print(f"キャップ上端: x={cap[0]*1000:.1f}  y={cap[1]*1000:.1f}  z={cap[2]*1000:.1f} mm", file=out)
print(f"置き面 {surface*1000:.0f} mm からの高さ: {r['height']*1000:.0f} mm   点数 {r['n']}", file=out)
print(f"J1軸からの水平距離: {np.hypot(cap[0], cap[1])*1000:.0f} mm", file=out)

if color is not None:
    ri = param.rgb_intrinsic
    Kc = np.array([[ri.fx, 0, ri.cx], [0, ri.fy, ri.cy], [0, 0, 1]])
    Tcb = np.linalg.inv(T)

    def proj(P3):
        q = P3 @ Tcb[:3, :3].T + Tcb[:3, 3]
        q = q[q[:, 2] > 0]
        return np.round((q[:, :2] / q[:, 2:3]) @ Kc[:2, :2].T + Kc[:2, 2]).astype(int)

    vis = color.copy()
    for (x, y) in proj(r["points"][::5]):
        if 0 <= x < vis.shape[1] and 0 <= y < vis.shape[0]:
            vis[y, x] = (0, 255, 0)
    cp = proj(np.array([cap]))
    if len(cp):
        cv2.drawMarker(vis, (int(cp[0][0]), int(cp[0][1])), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
    from piper_pp.camera import for_display
    cv2.imwrite(str(data_path("locate_result.jpg")), for_display(vis))
    print("確認画像: data/locate_result.jpg", file=out)
