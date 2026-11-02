#!/usr/bin/env python3
"""B の接続確認（P1）: ① MoveIt の tcp_link と A の TCP が一致するか ② 小さく動かして「MoveIt 完了」と「実機到達」の差を測る

使い方（agx_arm_ctrl＋MoveIt を起動した状態で）:
  python ros2/p1_check.py                 # ① だけ（動かさない）
  python ros2/p1_check.py --move 5        # ①＋② J1 を +5° → 0 に戻す（MoveIt で計画・実行）
  ※ どちらも末尾に --ros-args -r joint_states:=/feedback/joint_states を付ける
結果: 画面と data/p1_check.json（--move のときは data/segments_B_p1.csv にも記録）
合格の目安: ① 位置の差 1 mm 未満・姿勢の差 0.1° 未満  ② 実機到達が MoveIt 完了より何秒遅れるかを記録（数値は比較の前提になる）
"""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation as Rot

import rosutil  # noqa: F401
from rosutil import ARM_JOINTS, D, Feedback, offline_robot, spin_in_background
from piper_pp.config import CFG, data_path
from piper_pp import kinematics as K
from piper_pp.seglog import SegLog
from pymoveit2 import MoveIt2, MoveIt2State


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    node = Node("p1_check")
    cbg = ReentrantCallbackGroup()
    arm = MoveIt2(node, joint_names=ARM_JOINTS, base_link_name="base_link", end_effector_name="tcp_link",
                  group_name="arm", callback_group=cbg)
    arm.max_velocity = CFG["ros2"]["max_velocity_scaling"]
    arm.max_acceleration = CFG["ros2"]["max_acceleration_scaling"]
    fb = Feedback(node, callback_group=cbg)
    ex, _ = spin_in_background(node)
    fk = offline_robot()
    out = {}
    try:
        if not fb.wait_ready():
            raise SystemExit("/feedback/joint_states が来ない（agx_arm_ctrl を起動したか）")
        q, g, _ = fb.get()
        print("実機の関節角 [deg]:", [round(v / D, 2) for v in q], " グリッパ幅:", None if g is None else round(abs(g) * 1000, 1), "mm")
        # ① MoveIt（URDF）の tcp_link と、A の TCP（SDK の順運動学＋0.145 m）を比べる
        ps = arm.compute_fk(joint_state=q, timeout_sec=5.0)
        if isinstance(ps, list):
            ps = ps[0] if ps else None
        p_sdk, _ = K.tcp_of(fk, [v / D for v in q], CFG["tcp_z"])
        R_sdk = Rot.from_euler("xyz", fk.fk(q)[3:])
        if ps is None:
            print("① compute_fk が返らない（move_group が起動しているか確認）")
        else:
            p = ps.pose.position; o = ps.pose.orientation
            p_mv = np.array([p.x, p.y, p.z]); R_mv = Rot.from_quat([o.x, o.y, o.z, o.w])
            dp = float(np.linalg.norm(p_mv - p_sdk) * 1000)
            dr = float(np.degrees((R_mv.inv() * R_sdk).magnitude()))
            print(f"① tcp_link（MoveIt）  {np.round(p_mv * 1000, 1)} mm  frame={ps.header.frame_id}")
            print(f"   TCP（A, SDK＋0.145） {np.round(p_sdk * 1000, 1)} mm")
            print(f"   差: 位置 {dp:.2f} mm / 姿勢 {dr:.3f}°  → {'OK' if dp < 1.0 and dr < 0.1 else '要確認（tcp_offset の指定を確認）'}")
            out.update(tcp_pos_diff_mm=dp, tcp_rot_diff_deg=dr)
        # ② 小さく動かして遅れを測る
        if "--move" in argv:
            deg = float(argv[argv.index("--move") + 1])
            if abs(deg) > 10:
                raise SystemExit("--move は ±10° まで")
            log = SegLog("B_p1", fk); log.cycle = 1
            q0 = list(q)
            for name, dq in ((f"J1 {deg:+.0f}°", deg), ("J1 戻す", 0.0)):
                tgt = list(q0); tgt[0] = q0[0] + dq * D
                input(f"② {name} を MoveIt で計画・実行します。周囲を確認して Enter: ")
                t0 = time.perf_counter(); traj = arm.plan(joint_positions=tgt); pms = (time.perf_counter() - t0) * 1000
                if traj is None:
                    raise SystemExit("計画に失敗")
                last = traj.points[-1]; ts = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
                log.start(name, target_rad=tgt, planner="joint:" + CFG["ros2"]["planner_id"], plan_ms=pms,
                          traj_s=ts, n_points=len(traj.points))
                t1 = time.monotonic(); arm.execute(traj); time.sleep(0.1)
                while arm.query_state() != MoveIt2State.IDLE:
                    log.sample(fb.get()[0]); time.sleep(0.02)
                exec_s = time.monotonic() - t1
                ok, _ = fb.wait_arrival(tgt, CFG["ros2"]["arrive_tol_deg"], 20.0, log)
                arrive_s = time.monotonic() - t1
                row = log.end(ok=ok, exec_s=exec_s, arrive_s=arrive_s)
                print(f"   {name}: 計画 {pms:.0f} ms / 軌道 {ts:.2f} s / MoveIt 完了 {exec_s:.2f} s / 実機到達 {arrive_s:.2f} s"
                      f"（遅れ {arrive_s - exec_s:+.2f} s） 最終誤差 {row.get('final_err_deg', '')}°")
                out.setdefault("move", []).append(dict(name=name, plan_ms=pms, traj_s=ts, exec_s=exec_s, arrive_s=arrive_s,
                                                       lag_s=arrive_s - exec_s, final_err_deg=row.get("final_err_deg")))
        data_path("p1_check.json").write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
        print("記録: data/p1_check.json")
    except SystemExit as e:
        print(e)
    finally:
        ex.shutdown(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
