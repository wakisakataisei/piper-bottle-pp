"""動作区間ごとの記録（A と B で同じ形式）

1区間 = 1回の移動（look → pre など）。実機の関節角を動作中に記録し、A と B を同じ物差しで比べる。
- data/segments_<方式>.csv : 区間ごとの集計（計画時間、到達までの時間、実際の経路長、最終誤差など）
- data/traj_<方式>.csv     : 動作中の実際の関節角（約 20 Hz）。レポートで軌跡を描くのに使う
"""
import csv
import math
import time

import numpy as np

from .config import data_path, CFG
from .kinematics import tcp_of

D = 180 / math.pi
SEG_COLS = ["time", "method", "cycle", "segment", "planner", "plan_ms", "traj_s", "n_points", "exec_s", "arrive_s",
            "joint_path_deg", "tcp_path_mm", "tcp_straight_mm", "max_abs_j6_deg", "peak_joint_speed_dps",
            "final_err_deg", "ok", "note"]


class SegLog:
    def __init__(self, method, fk_robot):
        self.method = method
        self.robot = fk_robot          # fk() だけ使う（CAN 不要）
        self.cycle = 0
        self.seg_path = data_path(f"segments_{method}.csv")
        self.traj_path = data_path(f"traj_{method}.csv")
        for p, cols in ((self.seg_path, SEG_COLS), (self.traj_path, ["time", "method", "cycle", "segment", "t_s", "j1", "j2", "j3", "j4", "j5", "j6"])):
            if not p.exists():
                with open(p, "w", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow(cols)
        self._cur = None

    def start(self, segment, target_rad=None, planner="", plan_ms=None, traj_s=None, n_points=None, note=""):
        self._cur = dict(segment=segment, target=None if target_rad is None else list(target_rad), planner=planner,
                         plan_ms=plan_ms, traj_s=traj_s, n_points=n_points, note=note, t0=time.monotonic(), samples=[])

    def sample(self, q_rad, min_dt=0.04):
        """実際の関節角を記録（間隔は最短 min_dt 秒。速く呼ばれても間引く）"""
        if self._cur is None or q_rad is None:
            return
        t = time.monotonic() - self._cur["t0"]
        S = self._cur["samples"]
        if not S or t - S[-1][0] >= min_dt:
            S.append((t, list(q_rad)))

    def end(self, ok=True, exec_s=None, arrive_s=None):
        c = self._cur
        if c is None:
            return None
        self._cur = None
        t_end = time.monotonic() - c["t0"]
        S = c["samples"]
        stamp = time.strftime("%F %T")
        row = dict(time=stamp, method=self.method, cycle=self.cycle, segment=c["segment"], planner=c["planner"],
                   plan_ms=_r(c["plan_ms"], 1), traj_s=_r(c["traj_s"], 2), n_points=c["n_points"] or "",
                   exec_s=_r(exec_s, 2), arrive_s=_r(arrive_s if arrive_s is not None else t_end, 2), ok=int(bool(ok)), note=c["note"])
        if len(S) >= 2:
            Q = np.array([q for _, q in S]) * D
            t = np.array([s for s, _ in S])
            dq = np.abs(np.diff(Q, axis=0))
            row["joint_path_deg"] = round(float(dq.sum()), 1)
            dt = np.maximum(np.diff(t), 1e-3)
            row["peak_joint_speed_dps"] = round(float((dq.max(axis=1) / dt).max()), 1)
            row["max_abs_j6_deg"] = round(float(np.abs(Q[:, 5]).max()), 1)
            P = np.array([tcp_of(self.robot, q, CFG["tcp_z"])[0] for q in Q])
            row["tcp_path_mm"] = round(float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum() * 1000), 1)
            row["tcp_straight_mm"] = round(float(np.linalg.norm(P[-1] - P[0]) * 1000), 1)
            if c["target"] is not None:
                row["final_err_deg"] = round(float(np.abs(Q[-1] - np.array(c["target"]) * D).max()), 2)
            with open(self.traj_path, "a", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                for s, q in S:
                    w.writerow([stamp, self.method, self.cycle, c["segment"], round(s, 3)] + [round(v * D, 3) for v in q])
        with open(self.seg_path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([row.get(k, "") for k in SEG_COLS])
        return row


def _r(v, n):
    return "" if v is None else round(float(v), n)
