"""B（ROS2＋MoveIt）用の共通部品

- 実行は venv_pp の Python（piper_pp・pyAgxArm・ultralytics・pymoveit2 が入っている）で行う
- ROS2 の rclpy などは `source /opt/ros/humble/setup.bash` と `source ~/agx_arm_ws/install/setup.bash` で読み込む
"""
import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sensor_msgs.msg import JointState  # noqa: E402

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
D = math.pi / 180


def offline_robot():
    """順運動学（fk）だけに使う pyAgxArm の腕オブジェクト。connect() しないので CAN は使わない"""
    from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW
    cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can_piper")
    return AgxArmFactory.create_arm(cfg)


def spin_in_background(node, threads=4):
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(threads)
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    return ex, t


class Feedback:
    """/feedback/joint_states（agx_arm_ctrl が配信する実機の関節角）を受け取る"""
    def __init__(self, node, topic="/feedback/joint_states", callback_group=None):
        self.q = None            # rad, joint1〜6
        self.gripper = None      # m（グリッパの開き幅）
        self.stamp = 0.0
        self._lock = threading.Lock()
        node.create_subscription(JointState, topic, self._cb, 10, callback_group=callback_group)

    def _cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in ARM_JOINTS):
            with self._lock:
                self.q = [pos[n] for n in ARM_JOINTS]
                self.gripper = pos.get("gripper")
                self.stamp = time.monotonic()

    def get(self):
        with self._lock:
            return (None if self.q is None else list(self.q)), self.gripper, self.stamp

    def wait_ready(self, timeout=10.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.get()[0] is not None:
                return True
            time.sleep(0.05)
        return False

    def wait_arrival(self, target_rad, tol_deg, timeout, log=None, settle=0.3):
        """実機の関節角が目標の tol_deg 以内に入り、settle 秒そのままなら到達。戻り値: (到達したか, 経過秒)"""
        t0 = time.monotonic(); t_in = None
        while time.monotonic() - t0 < timeout:
            q, _, _ = self.get()
            if q is not None:
                if log is not None:
                    log.sample(q)
                err = max(abs(a - b) for a, b in zip(q, target_rad)) / D
                if err <= tol_deg:
                    t_in = t_in or time.monotonic()
                    if time.monotonic() - t_in >= settle:
                        if log is not None:
                            log.sample(q, min_dt=0.0)
                        return True, t_in - t0
                else:
                    t_in = None
            time.sleep(0.02)
        return False, time.monotonic() - t0
