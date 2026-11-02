"""実機・カメラなしで動く確認（pytest -q）
- kinematics: pyAgxArm の fk() はオフラインで計算できる（CAN 不要）
- handeye   : 既知の答えを持つ合成データで校正計算を検証
- vision    : 合成した円柱（ボトル）点群でキャップ位置推定を検証
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from piper_pp import kinematics as K          # noqa: E402
from piper_pp.handeye import solve_handeye     # noqa: E402

TCP_Z = 0.145


@pytest.fixture(scope="module")
def robot():
    pa = pytest.importorskip("pyAgxArm")
    cfg = pa.create_agx_arm_config(robot=pa.ArmModel.PIPER, firmeware_version=pa.PiperFW.DEFAULT, channel="can_piper")
    return pa.AgxArmFactory.create_arm(cfg)     # connect() しない


def test_zero_pose_points_5deg_up(robot):
    _, axis = K.tcp_of(robot, [0] * 6, TCP_Z)
    assert math.degrees(math.asin(axis[2])) == pytest.approx(5.0, abs=0.1)


def test_tool_down_when_sum_95(robot):
    _, axis = K.tcp_of(robot, [0, 60, -30, 0, 65, 0], TCP_Z)
    assert axis[2] == pytest.approx(-1.0, abs=1e-3)


@pytest.mark.parametrize("target", [(0.35, 0.0, 0.21), (0.30, 0.10, 0.20), (0.32, -0.12, 0.18)])
def test_ik_reaches_cap_height(robot, target):
    q, err, msg = K.solve(robot, target, 30, TCP_Z)
    assert q is not None, msg
    p, axis = K.tcp_of(robot, q, TCP_Z)
    assert np.linalg.norm(p - target) * 1000 < 1.0
    assert math.degrees(math.asin(-axis[2])) == pytest.approx(30.0, abs=0.2)


def test_ik_rejects_unreachable(robot):
    q, err, msg = K.solve(robot, (0.9, 0.0, 0.2), 30, TCP_Z)
    assert q is None


def test_handeye_synthetic():
    rng = np.random.default_rng(0)
    Xt = np.eye(4); Xt[:3, :3] = Rot.from_euler("xyz", [180, 5, 90], degrees=True).as_matrix(); Xt[:3, 3] = [0.03, -0.06, 0.05]
    Wt = np.eye(4); Wt[:3, 3] = [0.35, 0.0, 0.02]
    Tg, Tc = [], []
    for _ in range(12):
        e = [180 + rng.uniform(-20, 20), rng.uniform(-20, 20), 180 + rng.uniform(-30, 30)]
        G = np.eye(4); G[:3, :3] = Rot.from_euler("xyz", e, degrees=True).as_matrix()
        G[:3, 3] = [0.30 + rng.uniform(-.05, .05), rng.uniform(-.05, .05), 0.35 + rng.uniform(-.03, .03)]
        C = np.linalg.inv(G @ Xt) @ Wt; C[:3, 3] += rng.normal(0, 0.001, 3)
        Tg.append(G); Tc.append(C)
    X, P, spread = solve_handeye(Tg, Tc)
    assert np.linalg.norm(X[:3, 3] - Xt[:3, 3]) * 1000 < 3.0
    assert np.degrees(np.linalg.norm(Rot.from_matrix(X[:3, :3].T @ Xt[:3, :3]).as_rotvec())) < 0.5
    assert spread.mean() < 3.0


def test_locate_half_visible_cap():
    from piper_pp.vision import locate_in
    rng = np.random.default_rng(1)
    surface, cap_c, top = 0.021, np.array([0.35, 0.02]), 0.228
    cam = np.array([0.15, 0.0])                        # カメラは手前（J1軸側）
    pts = []
    # 胴体（半径 33 mm）と首・キャップ（半径 15 mm）の、カメラ側半分だけ見える
    for r, z0, z1 in [(0.033, surface, 0.19), (0.015, 0.19, top)]:
        th = rng.uniform(0, 2 * np.pi, 6000); z = rng.uniform(z0, z1, 6000)
        p = np.c_[cap_c[0] + r * np.cos(th), cap_c[1] + r * np.sin(th), z]
        facing = (p[:, :2] - cap_c) @ (cam - cap_c) > 0
        pts.append(p[facing])
    floor = np.c_[rng.uniform(0.2, 0.5, 5000), rng.uniform(-0.15, 0.15, 5000), np.full(5000, surface)]
    P = np.vstack(pts + [floor]) + rng.normal(0, 0.001, (len(np.vstack(pts + [floor])), 3))
    T = np.eye(4); T[:2, 3] = cam; T[2, 3] = 0.4
    r = locate_in(P, T, (0.35, 0.0), 0.30, surface)
    assert r is not None
    assert np.linalg.norm(np.array(r["cap"][:2]) - cap_c) * 1000 < 3.0
    assert abs(r["cap"][2] - top) * 1000 < 3.0


def test_yolo_box_filter_keeps_only_bottle():
    """V1: 枠の中に写る点だけ残る（ボトルの横の箱は落ちる）。YOLO 本体は使わない"""
    from piper_pp.vision_yolo import project, filter_by_boxes
    rng = np.random.default_rng(2)
    bottle = np.c_[0.35 + rng.normal(0, 0.01, 800), rng.normal(0, 0.01, 800), rng.uniform(0.05, 0.22, 800)]
    box = np.c_[0.35 + rng.normal(0, 0.01, 800), 0.12 + rng.normal(0, 0.01, 800), rng.uniform(0.03, 0.08, 800)]
    P = np.vstack([bottle, box])
    # カメラ: (0.15, 0, 0.45) から +x 方向を斜め下に見る
    T = np.eye(4)
    z = np.array([0.2, 0.0, -0.3]); z /= np.linalg.norm(z)
    x = np.array([0.0, -1.0, 0.0]); y = np.cross(z, x)
    T[:3, :3] = np.stack([x, y, z], 1); T[:3, 3] = [0.15, 0.0, 0.45]
    K = np.array([[488.1, 0, 314.5], [0, 488.1, 210.7], [0, 0, 1]])
    u, v, f = project(bottle, T, K)
    b = [dict(box=[u.min(), v.min(), u.max(), v.max()], conf=0.9)]
    kept = filter_by_boxes(P, T, K, b, margin=0.0)
    assert len(kept) >= 790
    assert (kept[:, 1] > 0.08).sum() < 20
    assert len(filter_by_boxes(P, T, K, [], margin=0.0)) == 0


def test_station_offsets():
    """台ごとの補正: zoff は数値でも {pick, place} でもよい。adv_pick が無ければ adv を使う"""
    from piper_pp.runlog import station_zoff, station_adv_pick, station_surface
    from piper_pp.config import CFG
    st = {"adv": {"A": 10.0}, "adv_pick": {"B": 20.0},
          "zoff": {"A": {"pick": 0.0, "place": -0.015}, "B": 0.01}, "surface": {"B": -0.0099}}
    assert station_zoff(st, "A", "pick") == 0.0 and station_zoff(st, "A", "place") == -0.015
    assert station_zoff(st, "B", "pick") == 0.01 and station_zoff(st, "B") == 0.01
    assert station_adv_pick(st, "A") == 10.0 and station_adv_pick(st, "B") == 20.0
    assert station_surface(st, "A") == CFG["surface_z"] and station_surface(st, "B") == -0.0099
