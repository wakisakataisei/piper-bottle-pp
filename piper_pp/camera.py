"""Orbbec Dabai DC1（pyorbbecsdk v1系）の取得補助

注意: カメラは同時に1プロセスしか開けない。他のスクリプトが開いていると
uvc_open ... return res-6（USB BUSY）になる。
"""
import time

import numpy as np
import cv2
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat

from .config import CFG


def quiet_sdk():
    """SDK のログ出力を減らす（対応していない版では何もしない）"""
    try:
        from pyorbbecsdk import Context, OBLogLevel
        Context.set_logger_level(OBLogLevel.ERROR)
    except Exception:
        pass


def start(depth=True, color=True):
    quiet_sdk()
    pipe = Pipeline(); cfg = Config()
    if depth:
        cfg.enable_stream(pipe.get_stream_profile_list(OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile())
    if color:
        cfg.enable_stream(pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_default_video_stream_profile())
    pipe.start(cfg)
    return pipe


def decode_color(c):
    buf = np.frombuffer(c.get_data(), np.uint8)
    if c.get_format() == OBFormat.MJPG:
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return cv2.cvtColor(buf.reshape(c.get_height(), c.get_width(), 3), cv2.COLOR_RGB2BGR)


def depth_m(d):
    """深度フレーム → [m] の配列（0 は無効）"""
    return np.frombuffer(d.get_data(), np.uint16).reshape(d.get_height(), d.get_width()) * d.get_depth_scale() / 1000.0


def for_display(img):
    """表示用の向きに直す（カメラを上下逆に取り付けているため 180° 回転）。計算には元画像を使う"""
    return cv2.rotate(img, cv2.ROTATE_180) if CFG.get("camera_display_rotate_180") else img


def depth_to_color_extrinsics(param):
    """深度カメラ → カラーカメラの回転と平行移動[m]。取れなければ単位行列"""
    try:
        R = np.array(param.transform.rot, float).reshape(3, 3)
        t = np.array(param.transform.transform, float).reshape(3) / 1000.0
        return R, t
    except Exception:
        return np.eye(3), np.zeros(3)


def grab(n_depth=8, need_color=True, timeout=5.0):
    """深度 n_depth 枚とカラー1枚を取得して返す: (depths[m], color, camera_param)"""
    pipe = start(depth=True, color=need_color)
    param = pipe.get_camera_param()
    depths, color, t0 = [], None, time.time()
    while (len(depths) < n_depth or (need_color and color is None)) and time.time() - t0 < timeout:
        fr = pipe.wait_for_frames(200)
        if fr is None:
            continue
        d = fr.get_depth_frame()
        if d is not None and len(depths) < n_depth:
            depths.append(depth_m(d))
        if need_color:
            c = fr.get_color_frame()
            if c is not None:
                color = decode_color(c)
    pipe.stop()
    return depths, color, param
