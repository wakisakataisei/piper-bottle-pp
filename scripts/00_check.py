"""接続と状態の確認（読むだけ。腕は動かさない）
使い方: python scripts/00_check.py
確認すること: ファームウェア、関節角が更新されている、arm_status=0（正常）、各関節のドライバ異常なし
"""
import time
import _bootstrap  # noqa: F401
from piper_pp.robot import connect, D

robot = connect()
fw = robot.get_firmware(timeout=2.0)
print("ファームウェア:", fw if fw else "取得できず（CAN を確認）")
if fw:
    print("  → config/robot.json の firmware:", "S-V1.8-2 以降は default" if fw.get("software_version", "") >= "S-V1.8" else "v183 などに合わせる")

for i in range(3):
    q = robot.get_joint_angles()
    print("関節[deg]:", "取得できず" if q is None else " ".join(f"J{k+1}={v/D:+7.2f}" for k, v in enumerate(q.msg)))
    time.sleep(0.3)

st = robot.get_arm_status()
if st is None:
    print("arm_status: 取得できず")
else:
    m = st.msg
    print(f"ctrl_mode={m.ctrl_mode!r} arm_status={m.arm_status!r} motion_status={m.motion_status!r} err_status={m.err_status!r}")

print("イネーブル状態:", robot.get_joints_enable_status_list())
for j in range(1, 7):
    s = robot.get_driver_states(j)
    if s is None:
        print(f"  J{j}: ドライバ状態なし"); continue
    f = s.msg.foc_status
    bad = [k for k in ("voltage_too_low", "motor_overheating", "driver_overcurrent", "driver_overheating",
                       "collision_status", "driver_error_status") if getattr(f, k, False)]
    print(f"  J{j}: {s.msg.vol:.1f} V  ドライバ{s.msg.foc_temp}°C  モータ{s.msg.motor_temp}°C  {'異常: ' + ','.join(bad) if bad else '異常なし'}")
robot.disconnect()
