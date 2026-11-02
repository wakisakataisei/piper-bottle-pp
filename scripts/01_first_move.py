"""初めての動作確認（速度10%）: 手首0 → 全関節0 → 前傾 → 全関節0、グリッパ開閉
使い方: python scripts/01_first_move.py
周囲に物や人がいないこと、非常停止に手が届くことを確認してから実行する。
電源投入時に関節が SDK 制限の外（例 J6=−121°、J4=−102.6°）にあると途中で止まる → 00_check.py と 00g_range.py
"""
import time
import _bootstrap  # noqa: F401
from piper_pp.robot import connect, enable, wait_motion, D
from piper_pp.config import CFG

robot = connect()
q0 = list(robot.get_joint_angles().msg)
print("現在の関節[deg]:", [round(v / D, 1) for v in q0])
input("周囲を確認。Enter でイネーブルして動かします（Ctrl+C で中止）: ")
enable(robot)
robot.set_speed_percent(CFG["speed_percent"])
grip = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)

wrist0 = q0[:3] + [0.0, 0.0, 0.0]
for name, q in [("手首0", wrist0), ("全関節0", [0] * 6), ("前傾", [0, 0.4, -0.4, 0, -0.4, 0]), ("全関節0", [0] * 6)]:
    robot.move_j(q)
    ok = wait_motion(robot, 20)
    now = robot.get_joint_angles().msg
    print(f"  {name}: {'到達' if ok else 'タイムアウト'}  実測[rad] {[round(v, 3) for v in now]}")
    if not ok:
        raise SystemExit("到達しません。00_check.py でエラー状態を確認してください")

grip.move_gripper_m(0.06, 1.0); time.sleep(1.5)
print("  グリッパ開:", grip.get_gripper_status().msg.value, "m（約0.06）")
grip.move_gripper_m(0.0, 1.0); time.sleep(1.5)
print("  グリッパ閉:", grip.get_gripper_status().msg.value, "m（約0）")
robot.disconnect()
print("完了（全関節0で保持）")
