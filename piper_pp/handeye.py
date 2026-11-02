"""ハンドアイ校正の計算（eye-in-hand、AX=XB）

OpenCV 5 系では calibrateHandEye が標準パッケージに含まれないため、numpy/scipy で実装している。
Park-Martin 法で初期値を求め、マーカー位置が全サンプルで一致するよう最小二乗で仕上げる。
"""
import numpy as np
from scipy.linalg import sqrtm
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot


def solve_handeye(Tg, Tc):
    """Tg: base→flange（4x4のリスト）、Tc: camera→marker（4x4のリスト）
    戻り値: X（flange→camera）、各サンプルのマーカー位置（base）、ばらつき[mm]"""
    n = len(Tg)
    M = np.zeros((3, 3)); C, d = [], []
    pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]
    for i, j in pairs:
        A = np.linalg.inv(Tg[j]) @ Tg[i]; B = Tc[j] @ np.linalg.inv(Tc[i])
        M += np.outer(Rot.from_matrix(B[:3, :3]).as_rotvec(), Rot.from_matrix(A[:3, :3]).as_rotvec())
    Rx = np.real(np.linalg.inv(sqrtm(M.T @ M)) @ M.T)
    for i, j in pairs:
        A = np.linalg.inv(Tg[j]) @ Tg[i]; B = Tc[j] @ np.linalg.inv(Tc[i])
        C.append(A[:3, :3] - np.eye(3)); d.append(Rx @ B[:3, 3] - A[:3, 3])
    tx = np.linalg.lstsq(np.vstack(C), np.hstack(d), rcond=None)[0]
    X0 = np.eye(4); X0[:3, :3] = Rx; X0[:3, 3] = tx
    W0 = np.mean([(Tg[i] @ X0 @ Tc[i])[:3, 3] for i in range(n)], axis=0)
    R0 = Tg[0] @ X0 @ Tc[0]

    def unpack(p):
        X = np.eye(4); X[:3, :3] = Rot.from_rotvec(p[:3]).as_matrix(); X[:3, 3] = p[3:6]
        W = np.eye(4); W[:3, :3] = Rot.from_rotvec(p[6:9]).as_matrix(); W[:3, 3] = p[9:12]
        return X, W

    def resid(p):
        X, W = unpack(p); r = []
        for i in range(n):
            E = np.linalg.inv(W) @ Tg[i] @ X @ Tc[i]
            r += list(E[:3, 3] * 1000) + list(Rot.from_matrix(E[:3, :3]).as_rotvec() * 100)
        return r

    p0 = np.r_[Rot.from_matrix(Rx).as_rotvec(), tx, Rot.from_matrix(R0[:3, :3]).as_rotvec(), W0]
    X, _ = unpack(least_squares(resid, p0).x)
    P = np.array([(Tg[i] @ X @ Tc[i])[:3, 3] for i in range(n)])
    return X, P, np.linalg.norm(P - P.mean(0), axis=1) * 1000
