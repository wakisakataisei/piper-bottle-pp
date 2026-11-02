"""キャップをつかんで 1 cm 持ち上げ、元に戻す試験（data/pick_trials.jsonl に記録）
使い方: python scripts/08_pick_test.py <キャップ上端 x> <y> <z> [mm] [--tilt 30] [--down 10] [--force 2] [--open 60] [--dry]
  --down  キャップ上端から何 mm 下を挟むか / --force 把持力 [N]（0〜3） / --open 開き幅 [mm]
"""
import sys
import time
import json
import numpy as np
import _bootstrap  # noqa: F401
from piper_pp.config import CFG, load_pose, data_path
from piper_pp.robot import connect, prepare, go, gripper_width_mm
from piper_pp import kinematics as K

TCP_Z = CFG["tcp_z"]
PK = CFG["pick"]
LIFT = 0.010
VALUED = ("--tilt", "--down", "--force", "--open")


def opt(name, default):
    return float(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default


args, skip = [], False
for a in sys.argv[1:]:
    if skip:
        skip = False; continue
    if a.startswith("--"):
        skip = a in VALUED; continue
    args.append(a)
if len(args) < 3:
    sys.exit(__doc__)
cap = np.array([float(v) for v in args[:3]]) / 1000.0
tilt = opt("--tilt", PK["tilt_deg"])
down = opt("--down", PK["grip_down"] * 1000) / 1000.0
force = opt("--force", PK["force_n"])
open_w = opt("--open", PK["open_width"] * 1000) / 1000.0

robot = connect()
grasp = cap - [0, 0, down]
qg, e, m = K.solve(robot, grasp, tilt, TCP_Z)
if qg is None:
    sys.exit(f"把持姿勢が解けません: {m}")
_, axis = K.tcp_of(robot, qg, TCP_Z)
qp, _, m = K.solve(robot, grasp - PK["pre"] * axis, tilt, TCP_Z, seed=qg)
ql, _, m2 = K.solve(robot, grasp + [0, 0, LIFT], tilt, TCP_Z, seed=qg)
if qp is None or ql is None:
    sys.exit(f"pre / lift が解けません: {m} {m2}")
print(f"把持点 {np.round(grasp*1000,1)} mm（キャップ上端から {down*1000:.0f} mm 下）、{tilt:.0f}° 下向き、力 {force} N")
for n, q in [("pre", qp), ("grasp", qg), ("lift +10mm", ql)]:
    print(f"  {n:10s}: {q}")
if "--dry" in sys.argv:
    robot.disconnect(); sys.exit(0)

look = load_pose("look")
input("\n周囲を確認。Enter で開始（Ctrl+C で中止）: ")
grip = prepare(robot)
go(robot, "look", look, deg=False)
grip.move_gripper_m(open_w, 1.0); time.sleep(1.5)
input("pre へ。Enter: "); go(robot, "pre", qp)
input("grasp へ（直進）。Enter: "); go(robot, "grasp", qg)
input("閉じます。Enter: ")
grip.move_gripper_m(0.0, force); time.sleep(2.0)
w = gripper_width_mm(grip); print(f"  閉じた後の幅: {w:.1f} mm")
if w is not None and w < 5:
    print("  幅がほぼ0 = 何も挟んでいない。開いて戻ります")
    grip.move_gripper_m(open_w, 1.0); time.sleep(1.5); go(robot, "pre", qp); go(robot, "look", look, deg=False)
    robot.disconnect(); sys.exit(1)
input("1 cm 持ち上げます（ボトルが落ちても良いよう下に物を置かない）。Enter: ")
go(robot, "lift", ql)
time.sleep(3.0)
w2 = gripper_width_mm(grip); print(f"  3秒保持後の幅: {w2:.1f} mm（閉じた直後 {w:.1f} mm）")
ans = input("【目視】ボトルは浮いていますか？ y=浮いた / n=残った・滑った: ").strip()
go(robot, "下ろす", qg)
grip.move_gripper_m(open_w, 1.0); time.sleep(1.5)
go(robot, "pre", qp); go(robot, "look", look, deg=False)
robot.disconnect()
rec = dict(t=time.strftime("%F %T"), cap=[float(v) for v in np.round(cap * 1000, 1)], down_mm=down * 1000, tilt=tilt,
           force=force, w_close=w, w_hold=w2, lifted=ans)
with open(data_path("pick_trials.jsonl"), "a", encoding="utf-8") as fp:
    fp.write(json.dumps(rec, ensure_ascii=False) + "\n")
print("記録: data/pick_trials.jsonl", rec)
