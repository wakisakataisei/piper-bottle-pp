"""対話ジョグ: 関節・直線移動、グリッパ開閉、姿勢の保存（data/poses.json）
使い方: python scripts/02_jog.py
保存が必要な姿勢: look（ボトルを上から見下ろす観察姿勢）。down（真下向き）は確認用で任意
"""
import time
import _bootstrap  # noqa: F401
from piper_pp.config import CFG, load_json, save_json
from piper_pp.robot import connect, prepare, wait_motion, check_j6, D

TCP_Z = CFG["tcp_z"]
MAX_DEG = 15.0      # 関節ジョグ 1回の上限 [deg]
MAX_LIN = 0.03      # 直線移動 1回の上限 [m]
Z_MIN = 0.02        # TCP がこれより低くなる直線移動は拒否 [m]

robot = connect()
grip = prepare(robot)


def show():
    q = robot.get_joint_angles().msg
    f = robot.get_flange_pose().msg
    t = robot.get_tcp_pose().msg
    g = grip.get_gripper_status()
    print("  関節[deg]:", " ".join(f"J{i+1}={v/D:+7.1f}" for i, v in enumerate(q)))
    print(f"  フランジ: x={f[0]:+.3f} y={f[1]:+.3f} z={f[2]:+.3f} m  r={f[3]/D:+6.1f} p={f[4]/D:+6.1f} y={f[5]/D:+6.1f} deg")
    print(f"  TCP     : x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} m  r={t[3]/D:+6.1f} p={t[4]/D:+6.1f} y={t[5]/D:+6.1f} deg")
    if g is not None:
        print(f"  グリッパ: 幅={g.msg.value*1000:5.1f} mm  力={g.msg.force:.2f} N")
    return q, f, t


def move_j(q, timeout=15):
    check_j6(q)
    robot.move_j(q)
    print("  到達" if wait_motion(robot, timeout) else "  タイムアウト（範囲外の可能性）")


HELP = """コマンド:
  <関節> <deg>   関節ジョグ（例 5 -10、最大15度）
  x|y|z <m>      TCPを向きそのままで直線移動（例 z -0.01、最大0.03 m）
  go <名前>      保存した姿勢へ関節移動（例 go look）
  o [mm] / c [N] グリッパ開（既定60 mm）/ 閉（既定 力1.0 N、最大3.0）
  s <名前>       現在の姿勢を保存（data/poses.json）
  p 現在値の表示（動かない） / h 全関節0 / ? ヘルプ / q 終了"""
print(HELP)
show()
while True:
    cmd = input("> ").strip().split()
    if not cmd:
        continue
    c0 = cmd[0]
    try:
        if c0 == "q":
            break
        if c0 == "?":
            print(HELP); continue
        if c0 == "p":
            show(); continue
        if c0 == "h":
            move_j([0] * 6, 20); show(); continue
        if c0 == "s" and len(cmd) == 2:
            q, f, t = show(); poses = load_json("poses.json")
            poses[cmd[1]] = {"joints": list(q), "flange": list(f), "tcp": list(t), "tcp_z": TCP_Z}
            save_json("poses.json", poses); print(f"  保存: {cmd[1]}"); continue
        if c0 == "go" and len(cmd) == 2:
            poses = load_json("poses.json")
            if cmd[1] not in poses:
                print("  保存済み:", list(poses)); continue
            move_j(poses[cmd[1]]["joints"], 20); show(); continue
        if c0 == "o":
            w = float(cmd[1]) / 1000 if len(cmd) > 1 else 0.06
            grip.move_gripper_m(min(max(w, 0.0), 0.07), 1.0); time.sleep(1.5); show(); continue
        if c0 == "c":
            n = float(cmd[1]) if len(cmd) > 1 else 1.0
            grip.move_gripper_m(0.0, min(max(n, 0.1), 3.0)); time.sleep(1.5); show(); continue
        if c0 in ("x", "y", "z") and len(cmd) == 2:
            d = float(cmd[1])
            if abs(d) > MAX_LIN:
                print(f"  1回の直線移動は ±{MAX_LIN} m までです"); continue
            t = list(robot.get_tcp_pose().msg)
            t["xyz".index(c0)] += d
            if t[2] < Z_MIN:
                print(f"  TCP z が {Z_MIN} m 未満になるため中止"); continue
            robot.move_l(robot.get_tcp2flange_pose(t))
            print("  到達" if wait_motion(robot, 15) else "  タイムアウト（IKが解けない可能性）"); show(); continue
        j, deg = int(c0), float(cmd[1])
        assert 1 <= j <= 6
        if abs(deg) > MAX_DEG:
            print(f"  1回の移動は ±{MAX_DEG}度 までです"); continue
        q = list(robot.get_joint_angles().msg); q[j - 1] += deg * D
        move_j(q); show()
    except (ValueError, IndexError, AssertionError):
        print("  入力形式が違います（? でヘルプ）")
    except SystemExit as e:
        print(" ", e)
robot.disconnect()
print("終了（腕はイネーブルのまま保持）")
