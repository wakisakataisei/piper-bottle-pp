"""認識の比較: V0（点群の形だけ）と V1（YOLO で選んでから点群）を、同じ撮影データで交互に実行する
使い方: python scripts/10_vision_compare.py [回数=10] [--label 名前] > /dev/null
  腕は look 姿勢で静止させておく（腕は動かさない）。台A・台B の登録（09 の --setup）が済んでいること。
  --label は条件の名前（例: base = ボトルだけ、clutter = 近くに別の物を置いた）。CSV に記録される。
結果: data/vision_compare.csv に1行ずつ追記、data/vision_<label>.jpg に1回目の検出結果画像
"""
import sys
import csv
import time
import numpy as np
import cv2
import _bootstrap  # noqa: F401
from piper_pp.config import CFG, data_path, load_json
from piper_pp.robot import connect
from piper_pp import vision, vision_yolo as VY
from piper_pp.camera import for_display

out = sys.stderr
n = int(next((a for a in sys.argv[1:] if a.isdigit()), "10"))
label = sys.argv[sys.argv.index("--label") + 1] if "--label" in sys.argv else "base"
st = load_json("stations.json")
if not {"A", "B"} <= set(st):
    sys.exit("先に scripts/09_pick_place.py --setup A / --setup B で台を登録してください")
R = CFG["pick"]["search_radius"]

print("YOLO を読み込み中（初回は重みをダウンロード）…", file=out)
VY.load_model()
robot = connect()
path = data_path("vision_compare.csv")
new = not path.exists()
fp = open(path, "a", newline="", encoding="utf-8"); wr = csv.writer(fp)
if new:
    wr.writerow(["time", "label", "round", "method", "station", "found", "cap_x_mm", "cap_y_mm", "cap_z_mm",
                 "height_mm", "n_points", "t_grab_ms", "t_method_ms", "yolo_ms", "n_boxes", "best_conf"])

for k in range(n):
    t0 = time.perf_counter()
    P, T, color, param = vision.points_in_base(robot, need_color=True)
    t_grab = (time.perf_counter() - t0) * 1000
    # V0
    t1 = time.perf_counter()
    r0 = {s: vision.locate_in(P, T, st[s][:2], R) for s in ("A", "B")}
    t_v0 = (time.perf_counter() - t1) * 1000
    # V1
    t2 = time.perf_counter()
    boxes, yms = VY.detect(color)
    K = VY.color_K(param)
    Pb = VY.filter_by_boxes(P, T, K, boxes)
    r1 = {s: (vision.locate_in(Pb, T, st[s][:2], R) if len(Pb) else None) for s in ("A", "B")}
    t_v1 = (time.perf_counter() - t2) * 1000
    best = max([b["conf"] for b in boxes], default=0.0)
    for method, res, tm, ym in (("V0", r0, t_v0, ""), ("V1", r1, t_v1, round(yms, 1))):
        for s in ("A", "B"):
            r = res[s]
            row = [time.strftime("%F %T"), label, k + 1, method, s, int(r is not None)]
            row += ([round(v * 1000, 1) for v in r["cap"]] + [round(r["height"] * 1000, 1), r["n"]]) if r else ["", "", "", "", ""]
            row += [round(t_grab, 1), round(tm, 1), ym, len(boxes) if method == "V1" else "", round(best, 3) if method == "V1" else ""]
            wr.writerow(row)
    fp.flush()
    f0 = {s: r0[s]["cap"] for s in r0 if r0[s]}; f1 = {s: r1[s]["cap"] for s in r1 if r1[s]}
    print(f"[{k+1}/{n}] 撮影 {t_grab:.0f} ms  V0 {t_v0:.0f} ms 検出 {list(f0)}  V1 {t_v1:.0f} ms（YOLO {yms:.0f}）枠 {len(boxes)} 検出 {list(f1)}", file=out)
    for s in set(f0) & set(f1):
        d = np.linalg.norm(np.array(f0[s]) - np.array(f1[s])) * 1000
        print(f"    台{s}: V0 と V1 の差 {d:.1f} mm", file=out)
    if k == 0 and color is not None:
        vis = color.copy()
        for b in boxes:
            x1, y1, x2, y2 = [int(v) for v in b["box"]]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 165, 255), 2)
        u, v, front = VY.project(np.array([c for c in f1.values()] or np.zeros((0, 3))), T, K)
        for uu, vv in zip(u, v):
            cv2.drawMarker(vis, (int(uu), int(vv)), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        vis = for_display(vis)
        for i, b in enumerate(boxes):
            cv2.putText(vis, f"bottle {b['conf']:.2f}", (10, 30 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.imwrite(str(data_path(f"vision_{label}.jpg")), vis)
        print(f"    画像: data/vision_{label}.jpg（橙=YOLO の枠、赤十字=V1 のキャップ）", file=out)
fp.close(); robot.disconnect()
print(f"記録: data/vision_compare.csv（{n} 回 × 2方式）", file=out)
