"""関節を脱力させる（全関節を脱力すると腕が落ちるので必ず手で支える）
使い方: python scripts/00d_disable.py [関節番号 1〜6、省略で全関節]
"""
import sys
import time
import _bootstrap  # noqa: F401
from piper_pp.robot import connect

j = int(sys.argv[1]) if len(sys.argv) > 1 else 255
robot = connect()
if j == 255:
    input("全関節を脱力します。腕を手で支えてから Enter（Ctrl+C で中止）: ")
for _ in range(3):           # 1回目が False のことがあるので数回試す
    if robot.disable(j):
        break
    time.sleep(0.2)
print("イネーブル状態:", robot.get_joints_enable_status_list())
robot.disconnect()
