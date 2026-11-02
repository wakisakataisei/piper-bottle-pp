"""V1: YOLO でボトルの枠を見つけ、枠の中に写る点だけで V0（点群の形）を実行する

V0 との違いは「どの点を使うか」だけ。キャップ位置の計算（vision.locate_in）は同じ。
- 物が複数あっても、ボトルと判定された物だけを使える
- カメラは上下逆に付いているので、YOLO には 180° 回した画像を渡し、枠を元の向きに戻す
"""
import time

import numpy as np

from .config import CFG, ROOT

Y = CFG["yolo"]
_model = None


def load_model():
    """Ultralytics YOLO を読み込む（初回は重み自動ダウンロード。社外配布時は AGPL-3.0 に注意）"""
    global _model
    if _model is None:
        from ultralytics import YOLO
        p = ROOT / Y["model"]          # リポジトリ直下にあればそれを使う（初回はカレントにダウンロードされる）
        _model = YOLO(str(p) if p.exists() else Y["model"])
    return _model


def detect(color, conf=None):
    """カラー画像 → ボトルの枠のリスト [dict(box=[x1,y1,x2,y2], conf)]（元の画像の向きの画素座標）と所要 ms"""
    import cv2
    t0 = time.perf_counter()
    rot = bool(CFG.get("camera_display_rotate_180"))
    img = cv2.rotate(color, cv2.ROTATE_180) if rot else color
    h, w = color.shape[:2]
    res = load_model().predict(img, classes=[Y["class_id"]], conf=conf or Y["conf"],
                               device=Y.get("device", "cuda"), verbose=False)[0]
    out = []
    for b in res.boxes:
        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
        if rot:   # 180° 回転を戻す
            x1, y1, x2, y2 = w - 1 - x2, h - 1 - y2, w - 1 - x1, h - 1 - y1
        out.append(dict(box=[x1, y1, x2, y2], conf=float(b.conf)))
    return out, (time.perf_counter() - t0) * 1000


def project(P_base, T_base_cam, K):
    """ロボット座標の点 → カラー画像の画素 (u, v) と、カメラの前にあるかどうか"""
    Tc = np.linalg.inv(T_base_cam)
    q = P_base @ Tc[:3, :3].T + Tc[:3, 3]
    front = q[:, 2] > 0.05
    z = np.where(front, q[:, 2], 1.0)
    u = K[0, 0] * q[:, 0] / z + K[0, 2]
    v = K[1, 1] * q[:, 1] / z + K[1, 2]
    return u, v, front


def filter_by_boxes(P_base, T_base_cam, K, boxes, margin=None):
    """枠（少し広げる）の中に写る点だけを残す。枠がなければ空"""
    if not boxes:
        return P_base[:0]
    margin = Y["box_margin"] if margin is None else margin
    u, v, front = project(P_base, T_base_cam, K)
    keep = np.zeros(len(P_base), bool)
    for d in boxes:
        x1, y1, x2, y2 = d["box"]
        mx, my = (x2 - x1) * margin, (y2 - y1) * margin
        keep |= front & (u >= x1 - mx) & (u <= x2 + mx) & (v >= y1 - my) & (v <= y2 + my)
    return P_base[keep]


def color_K(param):
    ri = param.rgb_intrinsic
    return np.array([[ri.fx, 0, ri.cx], [0, ri.fy, ri.cy], [0, 0, 1]], float)
