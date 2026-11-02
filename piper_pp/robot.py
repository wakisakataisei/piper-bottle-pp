"""PiPER への接続と共通の動作補助（pyAgxArm を使用）"""
import math
import time

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

from .config import CFG

FW = {"default": PiperFW.DEFAULT, "v183": PiperFW.V183, "v188": PiperFW.V188, "v189": PiperFW.V189}
D = math.pi / 180


def connect(channel=None):
    """CAN に接続して状態の受信を始める（腕は動かさない）"""
    cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=FW[CFG["firmware"]],
                                channel=channel or CFG["can_channel"])
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    time.sleep(0.5)
    return robot


def enable(robot, timeout=5.0):
    t0 = time.monotonic()
    while not robot.enable():
        if time.monotonic() - t0 > timeout:
            raise RuntimeError("enable に失敗。電源と非常停止を確認")
        time.sleep(0.01)


def wait_motion(robot, timeout=10.0):
    """motion_status が 0（到達）になるまで待つ。到達で True"""
    time.sleep(0.3)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = robot.get_arm_status()
        if st is not None and st.msg.motion_status == 0:
            return True
        time.sleep(0.02)
    return False


def prepare(robot):
    """イネーブル、速度、TCP、グリッパの初期化をまとめて行う"""
    enable(robot)
    robot.set_speed_percent(CFG["speed_percent"])
    robot.set_tcp_offset([0, 0, CFG["tcp_z"], 0, 0, 0])
    return robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)


def check_j6(q_rad):
    """カメラ配線保護のため J6 を制限する"""
    lim = CFG["j6_limit_deg"] * D
    if abs(q_rad[5]) > lim:
        raise SystemExit(f"J6={q_rad[5]/D:.1f}° が制限 ±{CFG['j6_limit_deg']}° を超えるため中止（カメラ配線保護）")


SEGLOG = None      # seglog.SegLog を入れると、go() の各区間を記録する（09 で使用）


def _wait_logged(robot, timeout, log):
    """wait_motion と同じ判定（motion_status==0）。待つ間、実機の関節角を約 20 Hz で記録する"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        q = robot.get_joint_angles()
        log.sample(None if q is None else q.msg)
        if time.monotonic() - t0 > 0.3:
            st = robot.get_arm_status()
            if st is not None and st.msg.motion_status == 0:
                q = robot.get_joint_angles()
                log.sample(None if q is None else q.msg, min_dt=0.0)   # 到達時の値を必ず残す
                return True
        time.sleep(0.05)
    return False


def go(robot, label, q, deg=True, timeout=25):
    """関節移動して到達を待つ。到達しなければその場で保持して終了"""
    q_rad = [a * D for a in q] if deg else list(q)
    check_j6(q_rad)
    if SEGLOG is not None:
        SEGLOG.start(label, target_rad=q_rad, planner="move_j")
    robot.move_j(q_rad)
    if SEGLOG is not None:
        ok = _wait_logged(robot, timeout, SEGLOG)
        SEGLOG.end(ok=ok)
    else:
        ok = wait_motion(robot, timeout)
    print(f"  {label}: {'到達' if ok else 'タイムアウト'}")
    if not ok:
        raise SystemExit("到達しないため中止（腕はその場で保持）")


def gripper_width_mm(grip):
    s = grip.get_gripper_status()
    return None if s is None else s.msg.value * 1000
