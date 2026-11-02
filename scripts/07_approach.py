"""キャップの手前まで近づく試験（つかまない）
使い方: python scripts/07_approach.py <キャップ上端 x> <y> <z> [mm] [--tilt 30] [--dry]
  --dry  : 関節角を計算して表示するだけ（腕は動かさない）
動作: look → 開く → 手前6 cm（pre）→ 手前2 cm（near、停止して目視）→ pre → look
"""
import sys
import time
import numpy as np
import _bootstrap  # noqa: F401
from piper_pp.config import CFG, load_pose
from piper_pp.robot import connect, prepare, go, D
from piper_pp import kinematics as K

TCP_Z = CFG["tcp_z"]
PK = CFG["pick"]
NEAR = 0.020
argv = sys.argv[1:]
tilt = PK["tilt_deg"]
if "--tilt" in argv:
    i = argv.index("--tilt"); tilt = float(argv[i + 1]); del argv[i:i + 2]
args = [a for a in argv if not a.startswith("--")]
if len(args) < 3:
    sys.exit(__doc__)
cap = np.array([float(v) for v in args[:3]]) / 1000.0

robot = connect()
grasp = cap - [0, 0, PK["grip_down"]]
qg, e, m = K.solve(robot, grasp, tilt, TCP_Z)
if qg is None:
    sys.exit(f"把持姿勢が解けません: {m}")
_, axis = K.tcp_of(robot, qg, TCP_Z)
plan = []
for name, back in [("pre（手前6cm）", PK["pre"]), ("near（手前2cm）", NEAR)]:
    q, e, m = K.solve(robot, grasp - back * axis, tilt, TCP_Z, seed=qg)
    if q is None:
        sys.exit(f"{name} が解けません: {m}")
    p, _ = K.tcp_of(robot, q, TCP_Z)
    if p[2] < 0.03:
        sys.exit(f"{name} の TCP 高さ {p[2]*1000:.0f} mm が低すぎます")
    plan.append((name, q, p))
print(f"キャップ上端 {np.round(cap*1000,1)} mm / 把持点 {np.round(grasp*1000,1)} mm / グリッパ {tilt:.0f}° 下向き")
for name, q, p in plan:
    print(f"  {name}: 関節 {q}  TCP {np.round(p*1000,1)} mm")
print(f"  grasp（参考、今回は行かない）: 関節 {qg}")
if "--dry" in sys.argv:
    robot.disconnect(); sys.exit(0)

look = load_pose("look")
input("\n周囲を確認。Enter で開始（Ctrl+C で中止）: ")
grip = prepare(robot)
go(robot, "look", look, deg=False)
grip.move_gripper_m(PK["open_width"], 1.0); time.sleep(1.5)
for name, q, p in plan:
    input(f"{name} へ移動します。Enter: ")
    go(robot, name, q)
    t = robot.get_tcp_pose().msg
    print(f"    実際のTCP {np.round(np.array(t[:3])*1000,1)} mm（計画 {np.round(p*1000,1)}）")
input("\n【目視】爪の間にキャップが来ているか、左右・上下のずれを確認。Enter で戻ります: ")
go(robot, "pre（戻り）", plan[0][1])
go(robot, "look", look, deg=False)
robot.disconnect()
print("完了（look で保持）")
