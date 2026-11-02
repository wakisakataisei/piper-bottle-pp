#!/usr/bin/env python3
"""今の実機の姿勢を data/poses.json に保存する（B 用。A の 02_jog.py の「s 名前」と同じ形式）
RViz の MotionPlanning で腕を動かしてから使う。同じ名前があれば <名前>_prev に退避する。
使い方: python ros2/save_pose.py look --ros-args -r joint_states:=/feedback/joint_states
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

import rosutil  # noqa: F401
from rosutil import D, Feedback, offline_robot, spin_in_background
from piper_pp.config import CFG, load_json, save_json
from piper_pp import kinematics as K


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    name = argv[0] if argv else "look"
    node = Node("save_pose"); fb = Feedback(node)
    ex, _ = spin_in_background(node)
    try:
        if not fb.wait_ready():
            raise SystemExit("/feedback/joint_states が来ない")
        q = fb.get()[0]
        q_deg = [v / D for v in q]
        if abs(q_deg[5]) > CFG["j6_limit_deg"]:
            raise SystemExit(f"J6 が {q_deg[5]:.0f}° で制限外。保存しません")
        fk = offline_robot()
        f = list(fk.fk(list(q)))
        t, ax = K.tcp_of(fk, q_deg, CFG["tcp_z"])
        poses = load_json("poses.json")
        if name in poses:
            poses[f"{name}_prev"] = poses[name]
        poses[name] = {"joints": list(q), "flange": f, "tcp": [float(v) for v in t], "tcp_z": CFG["tcp_z"]}
        save_json("poses.json", poses)
        down = float(np.degrees(np.arccos(np.clip(-np.asarray(ax)[2], -1, 1))))
        print(f"保存: {name}  関節 {np.round(q_deg, 1)} °")
        print(f"  TCP {np.round(np.asarray(t) * 1000, 0)} mm  手先の向きと真下のなす角 {down:.0f}°")
        if f"{name}_prev" in poses:
            print(f"  元の姿勢は {name}_prev に退避（戻すときは goto.py {name}_prev → save_pose.py {name}）")
    except SystemExit as e:
        print(e)
    finally:
        ex.shutdown(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
