"""J6 の原点を書き直す（00g_range.py で中心が大きくずれていた場合のみ）
使い方: python scripts/00h_calib_j6.py <00g_range.py の「中心」[deg]>

J6 だけ脱力させ、グリッパを手で範囲の中心に合わせてから calibrate_joint(6) を実行する。
書き込みは yes と入力したときだけ行う。実行後は 00g_range.py で中心が 0 付近になったか確認する。
"""
import sys
import time
import threading
import struct
import can
import _bootstrap  # noqa: F401
from piper_pp.config import CFG
from piper_pp.robot import connect

if len(sys.argv) < 2:
    sys.exit(__doc__)
CENTER = float(sys.argv[1])
TOL = 1.0
cur = {"j6": None}
stop = False


def reader():
    bus = can.interface.Bus(channel=CFG["can_channel"], interface="socketcan")
    while not stop:
        m = bus.recv(0.5)
        if m is not None and m.arbitration_id == 0x2A7:
            cur["j6"] = struct.unpack(">ii", bytes(m.data))[1] / 1000
    bus.shutdown()


def show():
    while not stop:
        if cur["j6"] is not None:
            d = cur["j6"] - CENTER
            mark = "OK" if abs(d) <= TOL else ("← 値を増やす向きへ" if d < 0 else "← 値を減らす向きへ")
            print(f"\rJ6={cur['j6']:+8.2f}  中心まで {d:+7.2f} deg  {mark}        ", end="", flush=True)
        time.sleep(0.1)


threading.Thread(target=reader, daemon=True).start()
robot = connect()
robot.disable(6)
print("J6 を脱力しました。イネーブル状態:", robot.get_joints_enable_status_list())
threading.Thread(target=show, daemon=True).start()
input(f"\nグリッパを回して『OK』表示にしたら Enter（目標 {CENTER:+.2f} deg）\n")
d = cur["j6"] - CENTER
if abs(d) > TOL:
    stop = True; sys.exit(f"\n中心から {d:+.2f} deg ずれているので中止。やり直してください")
if input("\n今の位置を J6 の原点(0°)として書き込みます。実行するなら yes と入力: ").strip() != "yes":
    stop = True; sys.exit("中止しました（何も書き込んでいません）")
print("calibrate_joint(6):", robot.calibrate_joint(6))
time.sleep(0.5)
print("clear_joint_error(6):", robot.clear_joint_error(6))
time.sleep(0.5)
stop = True
print(f"書き込み後の J6 = {cur['j6']:+.2f} deg（0 付近なら成功）")
print("J6 は脱力のままです。次に 00g_range.py で範囲を測り直してください")
robot.disconnect()
