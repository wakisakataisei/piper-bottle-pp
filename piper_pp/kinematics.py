"""把持姿勢の逆運動学（J4=0 の平面解）

グリッパを水平から tilt_deg だけ下に向け、J1軸から見て放射方向に TCP を目標点へ運ぶ関節角を求める。
順運動学は pyAgxArm の fk()（オフライン計算、CAN 通信なし）を使うので、実機の運動学と一致する。

実機で確認した性質:
- J2+J3+J5 = 5° でグリッパ水平、95° で真下（全関節0でグリッパは水平から5°上向き）
- J5 の SDK 制限 ±70° のため、真下向きは TCP 高さ約 0.08 m 以下でしか成立しない
"""
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

D = math.pi / 180
LIMITS = {1: (-150, 150), 2: (0, 180), 3: (-170, 0), 4: (-100, 100), 5: (-70, 70), 6: (-90, 90)}


def flange_matrix(pose):
    """[x,y,z,roll,pitch,yaw]（SDK の姿勢）→ 4x4 行列。SDK の姿勢角は extrinsic xyz（= Rz·Ry·Rx）"""
    T = np.eye(4)
    T[:3, :3] = Rot.from_euler("xyz", pose[3:]).as_matrix()
    T[:3, 3] = pose[:3]
    return T


def tcp_of(robot, q_deg, tcp_z):
    """関節角[deg] → (TCP位置[m], ツール軸の向き)"""
    f = robot.fk([a * D for a in q_deg])
    R = Rot.from_euler("xyz", f[3:]).as_matrix()
    return R @ [0, 0, tcp_z] + np.array(f[:3]), R[:, 2]


def solve(robot, target, tilt_deg, tcp_z, seed=(0, 60, -40, 0, 15, 0)):
    """target=[x,y,z][m] → (関節角[deg] or None, 位置誤差[mm], メッセージ)"""
    target = np.asarray(target, float)
    j1 = math.degrees(math.atan2(target[1], target[0]))
    s = 5 + tilt_deg

    def res(v):
        p, _ = tcp_of(robot, [j1, v[0], v[1], 0, s - v[0] - v[1], 0], tcp_z)
        return (p - target) * 1000

    best = None
    for j2_0, j3_0 in [(seed[1], seed[2]), (40, -30), (70, -60), (90, -80)]:
        r = least_squares(res, [j2_0, j3_0], bounds=([0, -170], [180, 0]))
        q = [j1, r.x[0], r.x[1], 0, s - r.x[0] - r.x[1], 0]
        err = float(np.linalg.norm(res(r.x)))
        if best is None or err < best[1]:
            best = (q, err)
    q, err = best
    for i, a in enumerate(q, start=1):
        lo, hi = LIMITS[i]
        if not lo <= a <= hi:
            return None, err, f"J{i}={a:.1f}° が範囲 {lo}〜{hi}° の外"
    if err > 2.0:
        return None, err, f"位置誤差 {err:.1f} mm（届かない）"
    return [round(float(a), 2) for a in q], err, "OK"
