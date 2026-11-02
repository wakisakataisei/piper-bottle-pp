#!/usr/bin/env python3
"""保存した姿勢へ MoveIt で移動する（B の準備用）
使い方: python ros2/goto.py look   … data/poses.json の look へ（home は全関節 0）
        python ros2/goto.py deg 0 60 -40 0 30 0 … 関節角[度]を直接指定
        python ros2/goto.py now  … 今の関節角を表示
        末尾に --ros-args -r joint_states:=/feedback/joint_states
"""
import sys
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

import rosutil  # noqa: F401
from rosutil import ARM_JOINTS, D, Feedback, spin_in_background
from piper_pp.config import CFG, load_pose
from pymoveit2 import MoveIt2, MoveIt2State


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    name = argv[0] if argv else "look"
    node = Node("goto_pose"); cbg = ReentrantCallbackGroup()
    arm = MoveIt2(node, joint_names=ARM_JOINTS, base_link_name="base_link", end_effector_name="tcp_link",
                  group_name="arm", callback_group=cbg)
    arm.planner_id = CFG["ros2"]["planner_id"]
    arm.max_velocity = CFG["ros2"]["max_velocity_scaling"]; arm.max_acceleration = CFG["ros2"]["max_acceleration_scaling"]
    fb = Feedback(node, callback_group=cbg)
    ex, _ = spin_in_background(node)
    try:
        if not fb.wait_ready():
            raise SystemExit("/feedback/joint_states が来ない")
        if name == "now":                      # 今の関節角を表示するだけ
            print("今の関節角 [deg]:", " ".join(f"{v / D:.1f}" for v in fb.get()[0])); return
        if name == "deg":                      # 関節角を直接指定: goto.py deg J1 J2 J3 J4 J5 J6（度）
            if len(argv) < 7:
                raise SystemExit("使い方: goto.py deg J1 J2 J3 J4 J5 J6（度）")
            q = [float(v) * D for v in argv[1:7]]
            print("今   [deg]:", " ".join(f"{v / D:6.1f}" for v in fb.get()[0]))
            print("目標 [deg]:", " ".join(f"{v / D:6.1f}" for v in q))
        else:
            q = [0.0] * 6 if name == "home" else list(load_pose(name))
        if abs(q[5]) > CFG["j6_limit_deg"] * D:
            raise SystemExit("J6 が制限外の姿勢です")
        traj = arm.plan(joint_positions=q)
        if traj is None:
            raise SystemExit("計画に失敗")
        input(f"{name} へ移動します（点 {len(traj.points)}）。周囲を確認して Enter: ")
        arm.execute(traj); time.sleep(0.1)
        while arm.query_state() != MoveIt2State.IDLE:
            time.sleep(0.05)
        ok, t = fb.wait_arrival(q, CFG["ros2"]["arrive_tol_deg"], 30.0)
        print(f"{name}: {'到達' if ok else '未到達'}（{t:.1f} s）")
    except SystemExit as e:
        print(e)
    finally:
        ex.shutdown(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
