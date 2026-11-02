"""認識と1サイクルの記録（A と B で共通）

- locate_stations(): look 姿勢で撮影し、台A・B のどちらにボトルがあるかとキャップ位置を返す（V0 / V1）
- CycleLog: data/cycles_<方式>.csv に1サイクル1行（A と B で同じ列）
"""
import csv
import time
from types import SimpleNamespace

import numpy as np

from .config import CFG, data_path
from . import vision

CYCLE_COLS = ["time", "method", "vision", "cycle", "from", "to", "cap_x", "cap_y", "cap_z", "locate_s", "yolo_ms",
              "plan_total_ms", "motion_s", "w_close", "w_lift", "result", "place_err_mm", "cycle_s", "note"]


class FlangeFromJoints:
    """ROS ノード用: 関節角から SDK と同じ順運動学でフランジ姿勢を返す（vision.points_in_base に渡す）"""
    def __init__(self, fk_robot, get_q):
        self.fk_robot, self.get_q = fk_robot, get_q

    def get_flange_pose(self):
        return SimpleNamespace(msg=self.fk_robot.fk(list(self.get_q())))


def locate_stations(robot, stations, use_yolo=False, radius=None):
    """戻り値: dict(found={A: r|None, B: r|None}, t_s, yolo_ms, boxes, color, T, K)"""
    t0 = time.perf_counter()
    P, T, color, param = vision.points_in_base(robot, need_color=use_yolo)
    return locate_from_points(P, T, color, param, stations, use_yolo, radius, t0)


def station_surface(stations, s):
    """台ごとの置き面の高さ [m]（stations.json の surface。無ければ robot.json の surface_z）"""
    return float(stations.get("surface", {}).get(s, CFG["surface_z"]))


def station_zoff(stations, s, kind="place"):
    """台ごとの上下の補正 [m]（stations.json の zoff。つかむ・置く高さに足す）"""
    v = stations.get("zoff", {}).get(s, 0.0)
    return float(v.get(kind, 0.0) if isinstance(v, dict) else v)


def station_adv_pick(stations, s):
    return float(stations.get("adv_pick", {}).get(s, stations.get("adv", {}).get(s, 0.0)))


def locate_from_points(P, T, color, param, stations, use_yolo=False, radius=None, t0=None):
    """撮影済みの点群（ロボット座標）から台A・B のキャップ位置を求める（camera_node でも使う）"""
    radius = radius or CFG["pick"]["search_radius"]
    t0 = time.perf_counter() if t0 is None else t0
    yolo_ms, boxes, K = None, [], None
    if use_yolo:
        from . import vision_yolo as VY
        boxes, yolo_ms = VY.detect(color)
        K = VY.color_K(param)
        P = VY.filter_by_boxes(P, T, K, boxes)
    found = {s: (vision.locate_in(P, T, stations[s][:2], radius, station_surface(stations, s)) if len(P) else None)
             for s in ("A", "B")}
    return dict(found=found, t_s=time.perf_counter() - t0, yolo_ms=yolo_ms, boxes=boxes, color=color, T=T, K=K)


class CycleLog:
    def __init__(self, method, vision_name):
        self.method, self.vision = method, vision_name
        self.path = data_path(f"cycles_{method}.csv")
        if not self.path.exists():
            with open(self.path, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(CYCLE_COLS)

    def write(self, **kw):
        kw.setdefault("time", time.strftime("%F %T"))
        kw.setdefault("method", self.method)
        kw.setdefault("vision", self.vision)
        cap = kw.pop("cap", None)
        if cap is not None:
            kw["cap_x"], kw["cap_y"], kw["cap_z"] = [round(float(v) * 1000, 1) for v in cap]
        row = []
        for k in CYCLE_COLS:
            v = kw.get(k, "")
            row.append("" if v is None else (round(v, 3) if isinstance(v, float) else v))
        with open(self.path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(row)


def place_error_mm(found_dst, dst_cap):
    if found_dst is None:
        return None
    return float(np.hypot(*(np.array(found_dst["cap"][:2]) - np.array(dst_cap[:2]))) * 1000)
