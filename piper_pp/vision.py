"""深度点群からボトルのキャップ位置を求める（学習なし）

手順: 深度を複数枚の中央値で平滑化 → カラーカメラ座標 → ハンドアイでロボット座標
→ 置き面（固定値）より上の点 → 範囲内の塊 → 最上部をキャップ上端
→ キャップは手前半分しか見えないので、半径既知の円を当てはめて中心を求める

実機で分かったこと:
- 置き面の高さを点の多さで自動判定すると誤る（浅い角度で見る面は点がまばら）→ 固定値を使う
- ほぼ水平に見る姿勢ではキャップ上端が約 6 mm 高く出る → 上から見下ろす look 姿勢を使う
"""
import numpy as np
from scipy.optimize import least_squares

from .config import CFG, data_path
from .kinematics import flange_matrix

V = CFG["vision"]


def load_handeye():
    p = data_path("T_flange_cam.npy")
    if not p.exists():
        raise SystemExit("data/T_flange_cam.npy がありません。scripts/05_handeye.py で校正してください")
    return np.load(p)


def base_from_camera(robot):
    """現在の姿勢でのロボット座標 ← カラーカメラ座標 の 4x4"""
    return flange_matrix(robot.get_flange_pose().msg) @ load_handeye()


def depth_points(depths, param):
    """深度[m]のリスト → カラーカメラ座標の点群[m]（N×3）"""
    di = param.depth_intrinsic
    D = np.stack(depths); D[D == 0] = np.nan
    with np.errstate(all="ignore"):
        Z = np.nanmedian(D, axis=0)
    h, w = Z.shape; v, u = np.mgrid[0:h, 0:w]
    ok = np.isfinite(Z) & (Z > CFG["camera_min_range"]) & (Z < 1.2)
    pd = np.stack([(u[ok] - di.cx) / di.fx * Z[ok], (v[ok] - di.cy) / di.fy * Z[ok], Z[ok]], axis=1)
    from . import camera
    R, t = camera.depth_to_color_extrinsics(param)
    return pd @ R.T + t


def points_in_base(robot, need_color=False):
    """撮影して、ロボット座標の点群・座標変換・カラー画像・カメラパラメータを返す"""
    from . import camera   # pyorbbecsdk はここで初めて読む（テストではカメラ不要）
    T = base_from_camera(robot)
    depths, color, param = camera.grab(V["n_frames"], need_color=need_color)
    if not depths:
        raise SystemExit("深度が取得できませんでした（カメラを他のプロセスが使っていないか確認）")
    pc = depth_points(depths, param)
    return pc @ T[:3, :3].T + T[:3, 3], T, color, param


def locate_in(P, T, center_xy, radius, surface=None):
    """center_xy の周り radius[m] でボトルを探す → dict(cap=[x,y,z], height, n) / 見つからなければ None"""
    surface = CFG["surface_z"] if surface is None else surface
    sel = np.hypot(P[:, 0] - center_xy[0], P[:, 1] - center_xy[1]) < radius
    B = P[sel & (P[:, 2] > surface + V["above_surface"])]
    if len(B) < 300:
        return None
    top = np.percentile(B[:, 2], 98)
    band = B[B[:, 2] > top - V["cap_band"]]
    c0 = np.median(band[:, :2], axis=0)
    away = (c0 - T[:2, 3]) / np.linalg.norm(c0 - T[:2, 3])
    c = least_squares(lambda c: np.linalg.norm(band[:, :2] - c, axis=1) - V["cap_radius"],
                      c0 + away * V["cap_radius"] / 2).x
    return dict(cap=[float(c[0]), float(c[1]), float(top)], height=float(top - surface), n=int(len(B)), points=B)
