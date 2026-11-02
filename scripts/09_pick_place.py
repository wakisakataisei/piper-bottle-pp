"""2つの台（A・B）の間でボトルを往復させるピック＆プレイス
使い方:
  python scripts/09_pick_place.py --setup A          … ボトルを台Aに置いて実行。台Aの位置を data/stations.json に記録
  python scripts/09_pick_place.py --setup B          … ボトルを台Bに置いて実行。台Bの位置を記録
  python scripts/09_pick_place.py --set-advance B 10 … 台Bで放射方向（J1軸から外向き）に 10 mm 前へ出してつかむ・置く
  python scripts/09_pick_place.py [回数] [--auto] [--yolo]  … 往復を実行
      --auto  各段階の Enter 待ちを省略
      --yolo  認識を V1（YOLO で選んでから点群）にする。省略時は V0（点群の形だけ）
結果: data/pp_trials.csv（従来の形式）、data/cycles_A.csv（B と共通の形式）、
      data/segments_A.csv・data/traj_A.csv（区間ごとの記録と実際の関節角）。失敗したらその場で止まる。
"""
import sys
import os
import csv
import time
import numpy as np
import _bootstrap  # noqa: F401
from piper_pp.config import CFG, load_pose, load_json, save_json, data_path
from piper_pp.robot import connect, prepare, go, gripper_width_mm
from piper_pp import robot as RB
from piper_pp import kinematics as K
from piper_pp import vision as L
from piper_pp.seglog import SegLog
from piper_pp.runlog import locate_stations, CycleLog, place_error_mm, station_surface, station_zoff, station_adv_pick

TCP_Z = CFG["tcp_z"]
SURFACE = CFG["surface_z"]
PK = CFG["pick"]
TILT, GRIP_DOWN, PRE, LIFT = PK["tilt_deg"], PK["grip_down"], PK["pre"], PK["lift"]
PLACE_GAP, FORCE, OPEN, SEARCH_R = PK["place_gap"], PK["force_n"], PK["open_width"], PK["search_radius"]
ST = "stations.json"

stations = load_json(ST)
adv = stations.get("adv", {})

if "--set-advance" in sys.argv:          # 腕に接続しない
    i = sys.argv.index("--set-advance"); name, mm = sys.argv[i + 1], float(sys.argv[i + 2])
    adv[name] = mm; stations["adv"] = adv; save_json(ST, stations)
    print(f"台{name} の前進補正を {mm} mm に設定（現在: {adv}）"); sys.exit(0)

robot = connect()
look = load_pose("look")


def ik(p, seed=None):
    q, e, m = K.solve(robot, np.array(p), TILT, TCP_Z, seed=seed or (0, 60, -40, 0, 15, 0))
    if q is None:
        raise SystemExit(f"IKが解けません {np.round(np.array(p)*1000,1)}: {m}")
    return q


def wait(msg):
    if "--auto" not in sys.argv:
        input(f"{msg} Enter: ")


def radial(xy):
    v = np.array(xy[:2], float); return v / np.linalg.norm(v)


grip = prepare(robot)

if "--setup" in sys.argv:
    name = sys.argv[sys.argv.index("--setup") + 1]
    go(robot, "look", look, deg=False)
    P, T, _, _ = L.points_in_base(robot)
    r = L.locate_in(P, T, (0.35, 0.0), 0.30, SURFACE)
    if r is None:
        sys.exit("ボトルが見つかりません")
    stations[name] = r["cap"]; save_json(ST, stations)
    print(f"台{name}: キャップ上端 {np.round(np.array(r['cap'])*1000,1)} mm（高さ {r['height']*1000:.0f} mm）を記録")
    robot.disconnect(); sys.exit(0)

if not {"A", "B"} <= set(stations):
    sys.exit("先に --setup A と --setup B を実行してください")
n_trials = int(next((a for a in sys.argv[1:] if a.isdigit()), "1"))
USE_YOLO = "--yolo" in sys.argv
if USE_YOLO:
    from piper_pp import vision_yolo as VY
    print("YOLO を読み込み中…"); VY.load_model()
RB.SEGLOG = SegLog("A", robot)
CLOG = CycleLog("A", "V1" if USE_YOLO else "V0")
csv_path = data_path("pp_trials.csv")
new = not os.path.exists(csv_path)
log = open(csv_path, "a", newline="", encoding="utf-8"); wr = csv.writer(log)
if new:
    wr.writerow(["time", "from", "to", "cap_x", "cap_y", "cap_z", "w_close", "w_lift", "result", "place_err_mm", "cycle_s"])

for k in range(n_trials):
    t0 = time.time()
    RB.SEGLOG.cycle = k + 1
    go(robot, "look", look, deg=False)
    loc = locate_stations(robot, stations, USE_YOLO)
    found = loc["found"]
    src = [s for s in ("A", "B") if found[s]]
    if len(src) != 1:
        print(f"ボトルが台A/Bの片方だけに見つかりません: {src}")
        CLOG.write(cycle=k + 1, locate_s=loc["t_s"], yolo_ms=loc["yolo_ms"], result="見つからず", note=str(src)); break
    src = src[0]; dst = "B" if src == "A" else "A"
    cap = np.array(found[src]["cap"])
    print(f"\n[{k+1}/{n_trials}] 台{src} → 台{dst}  キャップ {np.round(cap*1000,1)} mm  前進補正 {adv.get(src, 0.0)} mm")

    grasp = cap - [0, 0, GRIP_DOWN - station_zoff(stations, src, "pick")]
    grasp[:2] += station_adv_pick(stations, src) / 1000 * radial(cap)
    qg = ik(grasp); _, ax = K.tcp_of(robot, qg, TCP_Z)
    qp = ik(grasp - PRE * ax, qg); qlift = ik(grasp + [0, 0, LIFT], qg)
    dcap = np.array(stations[dst])
    place = np.r_[dcap[:2] + adv.get(dst, 0.0) / 1000 * radial(dcap), cap[2] - station_surface(stations, src) + station_surface(stations, dst) + station_zoff(stations, dst) - GRIP_DOWN + PLACE_GAP]
    qd_lift = ik(place + [0, 0, LIFT], qlift); qd = ik(place, qd_lift)
    _, axd = K.tcp_of(robot, qd, TCP_Z); qd_back = ik(place - PRE * axd, qd)

    grip.move_gripper_m(OPEN, 1.0); time.sleep(1.2)
    t_motion0 = time.time()
    wait("pre へ。"); go(robot, "pre", qp)
    wait("grasp へ。"); go(robot, "grasp", qg)
    grip.move_gripper_m(0.0, FORCE); time.sleep(1.5); w1 = gripper_width_mm(grip)
    print(f"  閉じた幅 {w1:.1f} mm")
    if w1 is None or w1 < 5:
        grip.move_gripper_m(OPEN, 1.0); time.sleep(1.2); go(robot, "pre", qp)
        wr.writerow([time.strftime("%F %T"), src, dst, *np.round(cap*1000, 1), w1, None, "つかめず", None, round(time.time()-t0, 1)]); log.flush()
        CLOG.write(cycle=k + 1, **{"from": src, "to": dst}, cap=cap, locate_s=loc["t_s"], yolo_ms=loc["yolo_ms"],
                   plan_total_ms=0.0, w_close=w1, result="つかめず", cycle_s=time.time() - t0)
        continue
    wait(f"{LIFT*100:.0f} cm 持ち上げます。"); go(robot, "lift", qlift)
    w2 = gripper_width_mm(grip); print(f"  持ち上げ後の幅 {w2:.1f} mm")
    wait(f"台{dst}の上へ運びます。"); go(robot, "transport", qd_lift)
    wait("下ろします。"); go(robot, "place", qd)
    grip.move_gripper_m(OPEN, 1.0); time.sleep(1.2)
    go(robot, "retreat", qd_back)
    motion_s = time.time() - t_motion0
    go(robot, "look2", look, deg=False)
    r = locate_stations(robot, stations, USE_YOLO)["found"][dst]
    err = place_error_mm(r, dcap)
    if r is None:
        result = "置き先に無い"
    else:
        result = "成功" if err < PK["success_err_mm"] else "ずれ大"
    dt = round(time.time() - t0, 1)
    print(f"  結果: {result}  置き位置のずれ {err if err is None else round(err,1)} mm  サイクル {dt} s")
    wr.writerow([time.strftime("%F %T"), src, dst, *np.round(cap*1000, 1), w1, w2, result, err if err is None else round(err, 1), dt]); log.flush()
    CLOG.write(cycle=k + 1, **{"from": src, "to": dst}, cap=cap, locate_s=loc["t_s"], yolo_ms=loc["yolo_ms"],
               plan_total_ms=0.0, motion_s=motion_s, w_close=w1, w_lift=w2, result=result,
               place_err_mm=err, cycle_s=time.time() - t0)
    if result != "成功":
        break

log.close(); robot.disconnect()
print("記録: data/pp_trials.csv, data/cycles_A.csv, data/segments_A.csv, data/traj_A.csv")
