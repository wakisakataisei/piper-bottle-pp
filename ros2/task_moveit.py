#!/usr/bin/env python3
"""B: MoveIt でボトルを台A↔台B に往復させる（A の 09_pick_place.py と同じ手順・同じ記録形式）

前提: agx_arm_ctrl＋MoveIt（start_single_agx_arm_moveit.launch.py）と vision_node.py が動いていること
使い方（venv_pp で、ROS2 と agx_arm_ws を source した端末）:
  python ros2/task_moveit.py [回数] [オプション] --ros-args -r joint_states:=/feedback/joint_states
オプション:
  --dry            計画だけして動かさない（各区間の計画時間と軌道を確認。最初に必ずこれ）
  --auto           各区間の前の Enter 待ちを省略
  --mode joint     関節角の目標（A と同じ目標を MoveIt が計画）※既定
  --mode pose      手先の位置・姿勢の目標（MoveIt の KDL が関節角を解く）。把持の接近・持上げ・置き・引きは直線
  --obstacle       台A・B の間に仮想の壁を置く（計画シーンだけ。実物は置かない）
  --wall-height 0.18  仮想の壁の高さ [m]
  --octomap        深度の点群（camera_node.py）を MoveIt の Octomap に取り込み、実物の障害物を避ける。
                   台の天面は点群から除き、つかむボトルは計画シーンの円柱で表して Octomap から外す
記録: data/cycles_B.csv, data/segments_B.csv, data/traj_B.csv（A と同じ列）
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation as Rot
from std_srvs.srv import Empty, Trigger
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory

import rosutil  # noqa: F401
from rosutil import ARM_JOINTS, D, Feedback, offline_robot, spin_in_background
from piper_pp.config import CFG, load_json, load_pose
from piper_pp import kinematics as K
from piper_pp.seglog import SegLog
from piper_pp.runlog import CycleLog, place_error_mm, station_surface, station_zoff, station_adv_pick
from pymoveit2 import MoveIt2, MoveIt2State

PK, RC = CFG["pick"], CFG["ros2"]
TCP_Z, SURF = CFG["tcp_z"], CFG["surface_z"]
TILT, GRIP_DOWN, PRE, LIFT = PK["tilt_deg"], PK["grip_down"], PK["pre"], PK["lift"]
PLACE_GAP, OPEN = PK["place_gap"], PK["open_width"]
TOUCH = ["gripper_base", "gripper_link1", "gripper_link2", "link6"]
SEARCH_J1 = (-5.0, -10.0, -15.0)   # 見つからないときに J1 を回す角度 [deg]（負 = 右 = 台Bの方向）
CAP_MARGIN = 0.04   # ボトルの円柱をキャップ上端からさらに上へ伸ばす量 [m]


def opt(argv, name, default):
    return argv[argv.index(name) + 1] if name in argv else default


class Task(Node):
    def __init__(self, argv):
        super().__init__("pick_place_moveit")
        self.argv = argv
        self.dry = "--dry" in argv
        self.auto = "--auto" in argv
        self.mode = opt(argv, "--mode", "joint")
        self.obstacle = "--obstacle" in argv
        self.wall_h = float(opt(argv, "--wall-height", "0.18"))
        self.octo = "--octomap" in argv
        cbg = ReentrantCallbackGroup()
        self.arm = MoveIt2(self, joint_names=ARM_JOINTS, base_link_name="base_link",
                           end_effector_name="tcp_link", group_name="arm", callback_group=cbg)
        self.arm.planner_id = RC["planner_id"]
        self.arm.max_velocity = RC["max_velocity_scaling"]
        self.arm.max_acceleration = RC["max_acceleration_scaling"]
        self.arm.allowed_planning_time = 2.0
        self.grip = MoveIt2(self, joint_names=["gripper"], base_link_name="base_link",
                            end_effector_name="gripper_base", group_name="gripper", callback_group=cbg)
        self.grip.max_velocity = 0.5
        self.grip.max_acceleration = 0.5
        self.fb = Feedback(self, callback_group=cbg)
        self.loc_cli = self.create_client(Trigger, "/bottle_vision/locate", callback_group=cbg)
        self.clear_cli = self.create_client(Empty, "/clear_octomap", callback_group=cbg)
        self.fk = offline_robot()
        self.log = SegLog("B", self.fk)
        self.plan_ms_total = 0.0
        self.pub_disp = self.create_publisher(DisplayTrajectory, "/display_planned_path", 1)

    # ---------- 共通 ----------
    def wait(self, msg):
        if not self.auto and not self.dry:
            input(f"{msg} Enter: ")

    def ik(self, p, seed=None):
        q, e, m = K.solve(self.fk, np.array(p), TILT, TCP_Z, seed=seed or (0, 60, -40, 0, 15, 0))
        if q is None:
            raise SystemExit(f"IKが解けません {np.round(np.array(p) * 1000, 1)}: {m}")
        return q

    def tcp_pose(self, q_deg):
        """関節角 → TCP の位置と姿勢（四元数 xyzw）。SDK の順運動学なので URDF の tcp_link と一致する"""
        f = self.fk.fk([a * D for a in q_deg])
        p, _ = K.tcp_of(self.fk, q_deg, TCP_Z)
        return [float(v) for v in p], [float(v) for v in Rot.from_euler("xyz", f[3:]).as_quat()]

    def locate(self):
        if not self.loc_cli.wait_for_service(timeout_sec=5.0):
            raise SystemExit("/bottle_vision/locate がない（vision_node.py を起動したか）")
        fut = self.loc_cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 20:
                raise SystemExit("認識が 20 秒で返らない")
            time.sleep(0.02)
        r = fut.result()
        out = json.loads(r.message)
        if not r.success:
            raise SystemExit(f"認識に失敗: {out}")
        return out

    def locate_retry(self, look_deg, prev, ok):
        """認識。ok(結果) が偽なら2回まで撮り直し、まだなら J1 を SEARCH_J1 だけ回して撮る。戻り値: (結果, 最後の姿勢)"""
        loc = self.locate()
        for _ in range(2):
            if ok(loc):
                return loc, prev
            print(f"  認識をやり直します（枠 {loc.get('n_boxes')} 個、信頼度 {loc.get('best_conf')}）")
            time.sleep(0.5); loc = self.locate()
        if not ok(loc):
            for dj1 in SEARCH_J1:
                q_s = list(look_deg); q_s[0] += dj1
                print(f"  J1 を {dj1:+.0f}° 回して探します")
                prev = self.move("search", q_deg=q_s, start=(prev if self.dry else None))
                loc = self.locate()
                if ok(loc):
                    break
        return loc, prev

    def check_traj(self, traj):
        """カメラ配線保護（J6）と J4 の範囲を、軌道の全点で確認"""
        idx = {n: i for i, n in enumerate(traj.joint_names)}
        j4 = max(abs(p.positions[idx["joint4"]]) for p in traj.points) / D
        j6 = max(abs(p.positions[idx["joint6"]]) for p in traj.points) / D
        ok = j6 <= CFG["j6_limit_deg"] and j4 <= RC["j4_limit_deg"]
        return ok, f"max|J4|={j4:.0f}° max|J6|={j6:.0f}°"

    def show(self, traj, start):
        """計画した軌道を RViz の MotionPlanning（Planned Path）に表示する"""
        q0 = start if start is not None else self.fb.get()[0]
        if q0 is None:
            return
        d = DisplayTrajectory()
        rs = RobotState(); rs.joint_state.name = list(ARM_JOINTS); rs.joint_state.position = [float(v) for v in q0]
        rt = RobotTrajectory(); rt.joint_trajectory = traj
        d.trajectory_start = rs; d.trajectory = [rt]
        self.pub_disp.publish(d)

    def plan(self, q_deg=None, pose=None, cartesian=False, start=None):
        """計画（制限を外れたら計画し直す）。戻り値: (軌道, 計画 ms の合計, 試行回数, メモ)"""
        total = 0.0; note = ""
        for k in range(RC["plan_retries"]):
            t0 = time.perf_counter()
            if q_deg is not None:
                traj = self.arm.plan(joint_positions=[a * D for a in q_deg], start_joint_state=start)
            else:
                traj = self.arm.plan(position=pose[0], quat_xyzw=pose[1], cartesian=cartesian,
                                     cartesian_fraction_threshold=0.95 if cartesian else 0.0, start_joint_state=start)
            total += (time.perf_counter() - t0) * 1000
            if traj is None or not traj.points:
                note = "計画失敗"; continue
            ok, note = self.check_traj(traj)
            if ok:
                return traj, total, k + 1, note
            note = "制限超え " + note
        return None, total, RC["plan_retries"], note

    def move(self, name, q_deg=None, pose=None, cartesian=False, start=None):
        planner = ("cartesian" if cartesian else ("pose:" if pose is not None else "joint:") + RC["planner_id"])
        traj, pms, tries, note = self.plan(q_deg, pose, cartesian, start)
        self.plan_ms_total += pms
        if traj is None:
            raise SystemExit(f"  {name}: 計画できません（{note}）。腕はその場で保持")
        last = traj.points[-1]
        idx = {n: i for i, n in enumerate(traj.joint_names)}
        target = [last.positions[idx[n]] for n in ARM_JOINTS]
        traj_s = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        print(f"  {name}: 計画 {pms:.0f} ms（{tries} 回）点 {len(traj.points)} 軌道 {traj_s:.2f} s  {note}")
        self.show(traj, start)
        if self.dry:
            time.sleep(traj_s + 0.8)      # RViz で1区間ずつ再生して見せる
            return target
        self.wait(f"  {name} を実行します。")
        self.log.start(name, target_rad=target, planner=planner, plan_ms=pms, traj_s=traj_s,
                       n_points=len(traj.points), note=f"{tries}回 {note}")
        t0 = time.monotonic()
        self.arm.execute(traj)
        time.sleep(0.1)
        while self.arm.query_state() != MoveIt2State.IDLE:
            q, _, _ = self.fb.get(); self.log.sample(q); time.sleep(0.02)
        exec_s = time.monotonic() - t0
        code = self.arm.get_last_execution_error_code()
        ok_exec = code is None or getattr(code, "val", 1) == 1
        ok, _ = self.fb.wait_arrival(target, RC["arrive_tol_deg"], RC["arrive_timeout_s"], self.log)
        arrive_s = time.monotonic() - t0
        self.log.end(ok=ok and ok_exec, exec_s=exec_s, arrive_s=arrive_s)
        print(f"    MoveIt 完了 {exec_s:.2f} s / 実機到達 {arrive_s:.2f} s（差 {arrive_s - exec_s:+.2f} s）")
        if not ok:
            raise SystemExit(f"  {name}: 実機が目標に届かない。腕はその場で保持")
        return target

    def gripper(self, width, label):
        if self.dry:
            return None
        traj = self.grip.plan(joint_positions=[width])
        if traj is None:
            raise SystemExit(f"グリッパの計画に失敗（{label}）")
        self.grip.execute(traj); time.sleep(0.1)
        while self.grip.query_state() != MoveIt2State.IDLE:
            time.sleep(0.02)
        t0 = time.monotonic(); last = None; stable = 0.0
        while time.monotonic() - t0 < 4.0:       # 実機の開き幅が落ち着くまで待つ
            _, g, _ = self.fb.get()
            if g is not None and last is not None and abs(g - last) < 0.0005:
                stable += 0.1
                if stable >= 0.6:
                    break
            else:
                stable = 0.0
            last = g; time.sleep(0.1)
        return None if last is None else abs(last) * 1000

    # ---------- 計画シーン ----------
    def setup_scene(self, st):
        a = self.arm
        a.remove_collision_object("bottle")
        a.add_collision_box(id="floor", size=(1.2, 1.2, 0.02), position=(0.2, 0.0, -0.011),
                            quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link")
        for s in ("A", "B"):
            x, y = st[s][:2]
            hb = max(station_surface(st, s), 0.006) - 0.004
            a.add_collision_box(id=f"stand_{s}", size=(0.12, 0.12, hb), position=(x, y, hb / 2),
                                quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link")
        if self.obstacle:
            pa, pb = np.array(st["A"][:2]), np.array(st["B"][:2])
            mid = (pa + pb) / 2; u = (pb - pa) / np.linalg.norm(pb - pa)
            q = Rot.from_euler("z", math.atan2(u[1], u[0])).as_quat()
            a.add_collision_box(id="wall", size=(0.02, 0.22, self.wall_h),
                                position=(float(mid[0]), float(mid[1]), self.wall_h / 2),
                                quat_xyzw=tuple(float(v) for v in q), frame_id="base_link")
        else:
            a.remove_collision_object("wall")
        time.sleep(1.0)

    def clear_octomap(self):
        if not self.clear_cli.wait_for_service(timeout_sec=3.0):
            print("  /clear_octomap がない（Octomap は使われていない）"); return
        fut = self.clear_cli.call_async(Empty.Request()); t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5:
            time.sleep(0.02)

    def prepare_octomap(self, cap, height):
        """つかむボトルを円柱で置き（Octomap から外れる）、接近中はぶつかってよいことにする。Octomap は撮り直す"""
        h = height + CAP_MARGIN        # 認識したキャップ上端より上にも伸ばす（キャップ高さの認識誤差でボトル上部が Octomap に残らないように）
        self.arm.add_collision_cylinder(id="bottle", height=h, radius=0.033,
                                        position=(float(cap[0]), float(cap[1]), float(cap[2] + CAP_MARGIN - h / 2)),
                                        quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link")
        time.sleep(0.3)
        self.arm.allow_collisions("bottle", True)
        self.clear_octomap()
        time.sleep(1.5)            # 点群 2 Hz・Octomap 更新 2 Hz を数回分待つ

    def attach_bottle(self, cap, height):
        if self.octo:
            self.arm.allow_collisions("bottle", False)      # 運ぶ間は台・障害物との衝突を見る（爪との接触は touch_links）
            time.sleep(0.3)
        h = height + CAP_MARGIN        # 認識したキャップ上端より上にも伸ばす（キャップ高さの認識誤差でボトル上部が Octomap に残らないように）
        self.arm.add_collision_cylinder(id="bottle", height=h, radius=0.033,
                                        position=(float(cap[0]), float(cap[1]), float(cap[2] + CAP_MARGIN - h / 2)),
                                        quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link")
        time.sleep(0.3)
        self.arm.attach_collision_object(id="bottle", link_name="tcp_link", touch_links=TOUCH)
        time.sleep(0.3)

    def release_bottle(self):
        self.arm.detach_collision_object("bottle"); time.sleep(0.3)
        if self.octo:
            # 置いたボトルは円柱のまま残す（消すと Octomap に点として入り、引くときに爪とぶつかる判定になる）
            self.arm.allow_collisions("bottle", True); time.sleep(0.3)
        else:
            self.arm.remove_collision_object("bottle"); time.sleep(0.3)

    # ---------- 1サイクル ----------
    def run(self, n):
        if not self.fb.wait_ready():
            raise SystemExit("/feedback/joint_states が来ない（agx_arm_ctrl を起動したか）")
        st = load_json("stations.json"); adv = st.get("adv", {})
        look = load_pose("look")
        look_deg = [v / D for v in look]
        self.setup_scene(st)
        if not self.octo and self.clear_cli.wait_for_service(timeout_sec=1.0):
            # camera_node が点群を出していると、つかむボトルが Octomap の障害物になり grasp が計画できない
            print("Octomap が有効（camera_node の点群）→ ボトルを円柱で置いて Octomap から外す（--octomap と同じ扱い）")
            self.octo = True
        radial = lambda xy: np.array(xy[:2], float) / np.linalg.norm(np.array(xy[:2], float))
        clog = None
        print(f"B: mode={self.mode} planner={RC['planner_id']} 仮想の壁={'あり' if self.obstacle else 'なし'} 点群={'あり' if self.octo else 'なし'} {'（計画のみ）' if self.dry else ''}")
        for k in range(n):
            t0 = time.time(); self.plan_ms_total = 0.0
            self.log.cycle = k + 1
            prev = self.move("look", q_deg=look_deg)
            loc, prev = self.locate_retry(look_deg, prev, lambda l: sum(1 for s_ in ("A", "B") if l[s_]) == 1)
            if clog is None:
                clog = CycleLog("B" if not self.dry else "B_dry", loc.get("vision", ""))
            src = [s for s in ("A", "B") if loc[s]]
            if len(src) != 1:
                print(f"ボトルが台A/Bの片方だけに見つかりません: {src}")
                clog.write(cycle=k + 1, locate_s=loc["t_s"], yolo_ms=loc.get("yolo_ms"), result="見つからず"); break
            src = src[0]; dst = "B" if src == "A" else "A"
            cap = np.array(loc[src]["cap"]); height = float(loc[src]["height"])
            print(f"\n[{k+1}/{n}] 台{src} → 台{dst}  キャップ {np.round(cap*1000,1)} mm  認識 {loc['t_s']*1000:.0f} ms")
            if self.octo:
                self.prepare_octomap(cap, height)
            grasp = cap - [0, 0, GRIP_DOWN - station_zoff(st, src, "pick")]
            grasp[:2] += station_adv_pick(st, src) / 1000 * radial(cap)
            qg = self.ik(grasp); _, ax = K.tcp_of(self.fk, qg, TCP_Z)
            qp = self.ik(grasp - PRE * ax, qg); ql = self.ik(grasp + [0, 0, LIFT], qg)
            dcap = np.array(st[dst])
            place = np.r_[dcap[:2] + adv.get(dst, 0.0) / 1000 * radial(dcap), cap[2] - station_surface(st, src) + station_surface(st, dst) + station_zoff(st, dst) - GRIP_DOWN + PLACE_GAP]
            qdl = self.ik(place + [0, 0, LIFT], ql); qd = self.ik(place, qdl)
            _, axd = K.tcp_of(self.fk, qd, TCP_Z); qdb = self.ik(place - PRE * axd, qd)
            P = self.mode == "pose"
            S = (lambda q: q) if self.dry else (lambda q: None)     # 計画のみのときは前の区間の終点から計画
            self.gripper(OPEN, "開く")
            t_m0 = time.time()
            prev = self.move("pre", q_deg=None if P else qp, pose=self.tcp_pose(qp) if P else None, start=S(prev))
            prev = self.move("grasp", q_deg=None if P else qg, pose=self.tcp_pose(qg) if P else None, cartesian=P, start=S(prev))
            w1 = self.gripper(0.0, "閉じる")
            if w1 is not None:
                print(f"  閉じた幅 {w1:.1f} mm")
                if w1 < 5:
                    self.gripper(OPEN, "開く"); self.move("pre", q_deg=qp)
                    clog.write(cycle=k + 1, **{"from": src, "to": dst}, cap=cap, locate_s=loc["t_s"], yolo_ms=loc.get("yolo_ms"),
                               plan_total_ms=self.plan_ms_total, w_close=w1, result="つかめず", cycle_s=time.time() - t0)
                    continue
            self.attach_bottle(cap, height)
            prev = self.move("lift", q_deg=None if P else ql, pose=self.tcp_pose(ql) if P else None, cartesian=P, start=S(prev))
            _, w2, _ = self.fb.get(); w2 = None if w2 is None else abs(w2) * 1000
            prev = self.move("transport", q_deg=None if P else qdl, pose=self.tcp_pose(qdl) if P else None, start=S(prev))
            prev = self.move("place", q_deg=None if P else qd, pose=self.tcp_pose(qd) if P else None, cartesian=P, start=S(prev))
            self.gripper(OPEN, "開く")
            self.release_bottle()
            prev = self.move("retreat", q_deg=None if P else qdb, pose=self.tcp_pose(qdb) if P else None, cartesian=P, start=S(prev))
            motion_s = time.time() - t_m0
            self.move("look2", q_deg=look_deg, start=S(prev))
            if self.dry:
                print(f"  計画の合計 {self.plan_ms_total:.0f} ms（計画のみ。動かしていない）")
                clog.write(cycle=k + 1, **{"from": src, "to": dst}, cap=cap, locate_s=loc["t_s"], yolo_ms=loc.get("yolo_ms"),
                           plan_total_ms=self.plan_ms_total, result="計画のみ", note=self.mode)
                break
            r = self.locate_retry(look_deg, prev, lambda l: l[dst] is not None)[0][dst]
            err = place_error_mm(r, dcap)
            result = "置き先に無い" if r is None else ("成功" if err < PK["success_err_mm"] else "ずれ大")
            dt = time.time() - t0
            print(f"  結果: {result}  置き位置のずれ {None if err is None else round(err, 1)} mm  サイクル {dt:.1f} s  計画の合計 {self.plan_ms_total:.0f} ms")
            clog.write(cycle=k + 1, **{"from": src, "to": dst}, cap=cap, locate_s=loc["t_s"], yolo_ms=loc.get("yolo_ms"),
                       plan_total_ms=self.plan_ms_total, motion_s=motion_s, w_close=w1, w_lift=w2, result=result,
                       place_err_mm=err, cycle_s=dt, note=f"{self.mode}{' wall' if self.obstacle else ''}{' octomap' if self.octo else ''}")
            if result == "置き先に無い":
                break
        print("記録: data/cycles_B.csv, data/segments_B.csv, data/traj_B.csv")


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    node = Task(argv)
    ex, _ = spin_in_background(node)
    n = int(next((a for a in argv if a.isdigit()), "1"))
    try:
        node.run(n)
    except SystemExit as e:
        print(e)
    except KeyboardInterrupt:
        print("中断（腕はその場で保持）")
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
