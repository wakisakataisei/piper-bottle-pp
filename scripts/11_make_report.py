"""A（Python）と B（ROS2＋MoveIt）の比較レポートを data/ の記録から作る
使い方: python scripts/11_make_report.py [--title "..."] [--pdf]
入力（あるものだけ使う）:
  data/vision_compare.csv          認識 V0 / V1 の比較（10_vision_compare.py）
  data/cycles_A.csv, cycles_B.csv  1サイクルごと（09 / task_moveit.py）
  data/segments_A.csv, segments_B.csv, traj_A.csv, traj_B.csv  区間ごと・実際の関節角
  data/p1_check.json               B の接続確認（p1_check.py）
  data/report_notes.md             所見（手で書く。見出し「## 」ごとにレポートへ入る）
出力: docs/report_A_vs_B.html（画像埋め込みの1ファイル）。--pdf で docs/report_A_vs_B.pdf も（Playwright があれば）
"""
import base64
import csv
import html
import io
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import _bootstrap  # noqa: F401
from piper_pp.config import CFG, ROOT, data_path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for f in font_manager.findSystemFonts():
    if "NotoSansCJK" in f or "NotoSansJP" in f:
        try:
            font_manager.fontManager.addfont(f)
        except Exception:
            pass
_jp = [f.name for f in font_manager.fontManager.ttflist if "CJK JP" in f.name or f.name == "Noto Sans JP"]
plt.rcParams["font.family"] = _jp[0] if _jp else "sans-serif"
plt.rcParams["axes.unicode_minus"] = False
CA, CB, C0, C1 = "#1f5fa8", "#a8520f", "#5c636b", "#3f7a1a"
SEG_ORDER = ["look", "pre", "grasp", "lift", "transport", "place", "retreat", "look2"]


def read_csv(name):
    p = data_path(name)
    if not p.exists():
        return []
    with open(p, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def col(rows, k, cond=None):
    return np.array([num(r.get(k)) for r in rows if cond is None or cond(r)], float)


def stat(a):
    a = a[np.isfinite(a)]
    if len(a) == 0:
        return "—"
    if len(a) == 1:
        return f"{a[0]:.2f}"
    return f"{a.mean():.2f} ± {a.std(ddof=1):.2f}"


def fig_b64(fig):
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=110, bbox_inches="tight", facecolor="white"); plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def table(head, rows):
    h = "".join(f"<th>{html.escape(str(x))}</th>" for x in head)
    b = "".join("<tr>" + "".join(f"<td>{html.escape(str(x))}</td>" for x in r) + "</tr>" for r in rows)
    return f"<table><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table>"


# ---------------- 各節 ----------------
def sec_conditions():
    rc, pk = CFG["ros2"], CFG["pick"]
    rows = [["速度", f"A: speed_percent {CFG['speed_percent']}%", f"B: MoveIt 速度・加速度スケール {rc['max_velocity_scaling']}・{rc['max_acceleration_scaling']}（実機の追従上限は起動時の speed_percent）"],
            ["計画", "なし（自前 IK の関節角へ move_j）", f"MoveIt {rc['planner_id']}（OMPL）。制限 |J6|≤{CFG['j6_limit_deg']}°・|J4|≤{rc['j4_limit_deg']}° を外れたら計画し直し"],
            ["到達の判定", "コントローラの motion_status=0", f"実機の関節角が目標の {rc['arrive_tol_deg']}° 以内に 0.3 秒"],
            ["把持", f"キャップ上端 −{pk['grip_down']*1000:.0f} mm、{pk['tilt_deg']:.0f}° 下向き、{pk['force_n']} N", "同じ（目標の関節角も同じ自前 IK）"]]
    return "<h2>1. 条件</h2>" + table(["項目", "A　Python（pyAgxArm）", "B　ROS2＋MoveIt"], rows)


def sec_p1():
    p = data_path("p1_check.json")
    if not p.exists():
        return ""
    d = json.loads(p.read_text(encoding="utf-8"))
    rows = []
    if "tcp_pos_diff_mm" in d:
        rows.append(["tcp_link（MoveIt）と A の TCP の差", f"位置 {d['tcp_pos_diff_mm']:.2f} mm / 姿勢 {d['tcp_rot_diff_deg']:.3f}°"])
    for m in d.get("move", []):
        rows.append([f"小動作 {m['name']}", f"計画 {m['plan_ms']:.0f} ms、軌道 {m['traj_s']:.2f} s、MoveIt 完了 {m['exec_s']:.2f} s、実機到達 {m['arrive_s']:.2f} s（遅れ {m['lag_s']:+.2f} s）"])
    return "<h2>2. B の接続確認（P1）</h2>" + table(["確認", "結果"], rows)


def sec_vision():
    R = read_csv("vision_compare.csv")
    if not R:
        return "<h2>3. 認識の比較（V0 / V1）</h2><p class='na'>data/vision_compare.csv がない（10_vision_compare.py を実行）</p>"
    out = ["<h2>3. 認識の比較（V0：点群の形だけ ／ V1：YOLO で選んでから点群）</h2>"]
    rows, labels = [], sorted(set(r["label"] for r in R))
    figs = []
    for lab in labels:
        for st in ("A", "B"):
            for m in ("V0", "V1"):
                S = [r for r in R if r["label"] == lab and r["station"] == st and r["method"] == m]
                if not S:
                    continue
                F = [r for r in S if r["found"] == "1"]
                rate = len(F) / len(S)
                if not F:
                    rows.append([lab, st, m, f"{len(F)}/{len(S)}", "—", "—", "—", "—"]); continue
                X = np.array([[num(r["cap_x_mm"]), num(r["cap_y_mm"]), num(r["cap_z_mm"])] for r in F])
                sd = X.std(axis=0, ddof=1) if len(X) > 1 else np.zeros(3)
                rows.append([lab, st, m, f"{len(F)}/{len(S)}（{rate*100:.0f}%）",
                             f"{X[:,0].mean():.1f}, {X[:,1].mean():.1f}, {X[:,2].mean():.1f}",
                             f"{sd[0]:.2f} / {sd[1]:.2f} / {sd[2]:.2f}", stat(col(S, "t_method_ms")),
                             stat(col(S, "yolo_ms")) if m == "V1" else "—"])
    out.append(table(["条件", "台", "方式", "検出", "キャップ平均 x, y, z [mm]", "ばらつき σx/σy/σz [mm]", "処理 [ms]", "うち YOLO [ms]"], rows))
    # V0 と V1 の差（同じ撮影データ）
    diffs = []
    for lab in labels:
        for st in ("A", "B"):
            by = {}
            for r in R:
                if r["label"] == lab and r["station"] == st and r["found"] == "1":
                    by.setdefault(r["round"], {})[r["method"]] = np.array([num(r["cap_x_mm"]), num(r["cap_y_mm"]), num(r["cap_z_mm"])])
            d = [np.linalg.norm(v["V0"] - v["V1"]) for v in by.values() if "V0" in v and "V1" in v]
            if d:
                diffs.append([lab, st, len(d), f"{np.mean(d):.2f}", f"{np.max(d):.2f}"])
    if diffs:
        out.append("<h3>同じ撮影データでの V0 と V1 の差</h3>" + table(["条件", "台", "回数", "平均 [mm]", "最大 [mm]"], diffs))
    # 図: 上から見たキャップ位置
    fig, axs = plt.subplots(1, len(labels), figsize=(4.6 * len(labels), 3.8), squeeze=False)
    for ax, lab in zip(axs[0], labels):
        for m, c, mk in (("V0", C0, "o"), ("V1", C1, "x")):
            F = [r for r in R if r["label"] == lab and r["method"] == m and r["found"] == "1"]
            if F:
                ax.plot(col(F, "cap_x_mm"), col(F, "cap_y_mm"), mk, color=c, label=m, ms=6, mfc="none" if mk == "o" else None)
        ax.set_title(f"条件: {lab}", fontsize=10); ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
        ax.set_aspect("equal", adjustable="datalim")
    out.append(f"<figure><img src='{fig_b64(fig)}'><figcaption>上から見たキャップ位置（各回）。点が集まっているほど再現性が高い</figcaption></figure>")
    imgs = [p for p in sorted(data_path(".").glob("vision_*.jpg"))]
    for p in imgs[:3]:
        out.append(f"<figure><img src='data:image/jpeg;base64,{base64.b64encode(p.read_bytes()).decode()}' style='max-width:520px'><figcaption>{html.escape(p.name)}（橙=YOLO の枠、赤十字=V1 のキャップ）</figcaption></figure>")
    return "".join(out)


def sec_cycles():
    A, B = read_csv("cycles_A.csv"), read_csv("cycles_B.csv")
    if not A and not B:
        return "<h2>4. サイクルの比較</h2><p class='na'>cycles_A.csv / cycles_B.csv がない</p>"
    rows = []
    for name, R in (("A", A), ("B", B)):
        for v in sorted(set(r["vision"] for r in R)):
            S = [r for r in R if r["vision"] == v]
            ok = [r for r in S if r["result"] == "成功"]
            rows.append([name, v, f"{len(ok)}/{len(S)}", stat(col(ok, "cycle_s")), stat(col(ok, "motion_s")), stat(col(S, "locate_s")),
                         stat(col(S, "plan_total_ms")), stat(col(ok, "place_err_mm")),
                         f"{np.nanmax(col(ok, 'place_err_mm')):.1f}" if ok else "—"])
    out = ["<h2>4. サイクルの比較（台A↔台B 往復）</h2>",
           table(["方式", "認識", "成功", "サイクル [s]", "動作のみ [s]", "認識 [s]", "計画の合計 [ms]", "置きずれ [mm]", "置きずれ最大"], rows),
           "<p class='note'>平均 ± 標準偏差。サイクル = look から確認の撮影まで。動作のみ = pre から retreat まで（認識・確認を除く）。A は --auto で実行した回を比べること。</p>"]
    fig, ax = plt.subplots(figsize=(7.5, 3.2))
    for i, (name, R, c) in enumerate((("A", A, CA), ("B", B, CB))):
        ok = [r for r in R if r["result"] == "成功"]
        if ok:
            y = col(ok, "cycle_s"); ax.plot(np.arange(1, len(y) + 1), y, "o-", color=c, label=f"{name}（{ok[0]['vision']}）")
    ax.set_xlabel("サイクル"); ax.set_ylabel("サイクル時間 [s]"); ax.grid(alpha=.3); ax.legend()
    out.append(f"<figure><img src='{fig_b64(fig)}'><figcaption>成功したサイクルの時間の推移</figcaption></figure>")
    return "".join(out)


def sec_segments():
    A, B = read_csv("segments_A.csv"), read_csv("segments_B.csv")
    if not A and not B:
        return ""
    segs = [s for s in SEG_ORDER if any(r["segment"] == s for r in A + B)]
    rows = []
    for s in segs:
        a = [r for r in A if r["segment"] == s and r["ok"] == "1"]; b = [r for r in B if r["segment"] == s and r["ok"] == "1"]
        rows.append([s, stat(col(a, "arrive_s")), stat(col(b, "arrive_s")), stat(col(b, "exec_s")), stat(col(b, "plan_ms")),
                     stat(col(a, "tcp_path_mm")), stat(col(b, "tcp_path_mm")), stat(col(a, "joint_path_deg")), stat(col(b, "joint_path_deg")),
                     stat(col(a, "final_err_deg")), stat(col(b, "final_err_deg"))])
    out = ["<h2>5. 区間ごとの比較</h2>",
           table(["区間", "A 到達 [s]", "B 到達 [s]", "B MoveIt 完了 [s]", "B 計画 [ms]", "A 手先の道のり [mm]", "B 手先の道のり [mm]",
                  "A 関節の動き [°]", "B 関節の動き [°]", "A 最終誤差 [°]", "B 最終誤差 [°]"], rows),
           "<p class='note'>到達 = 指令から実機が目標に入るまで。手先の道のり = 実機の関節角から順運動学で求めた TCP の移動距離（直線より長いほど遠回り）。</p>"]
    x = np.arange(len(segs)); w = 0.38
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.4))
    for k, (key, lab) in enumerate((("arrive_s", "到達までの時間 [s]"), ("tcp_path_mm", "手先の道のり [mm]"))):
        for j, (R, c, n) in enumerate(((A, CA, "A"), (B, CB, "B"))):
            m = [np.nanmean(col([r for r in R if r["segment"] == s and r["ok"] == "1"], key)) if R else np.nan for s in segs]
            axs[k].bar(x + (j - .5) * w, m, w, color=c, label=n)
        axs[k].set_xticks(x); axs[k].set_xticklabels(segs, rotation=30, fontsize=8); axs[k].set_ylabel(lab); axs[k].grid(axis="y", alpha=.3); axs[k].legend()
    out.append(f"<figure><img src='{fig_b64(fig)}'><figcaption>区間ごとの平均</figcaption></figure>")
    if B:
        lag = col(B, "arrive_s") - col(B, "exec_s")
        lag = lag[np.isfinite(lag)]
        if len(lag):
            out.append(f"<p>B で「MoveIt 完了」から「実機到達」までの遅れ: 平均 {lag.mean():.2f} s、最大 {lag.max():.2f} s（{len(lag)} 区間）。</p>")
    return "".join(out)


def sec_traj():
    TA, TB = read_csv("traj_A.csv"), read_csv("traj_B.csv")
    if not TA and not TB:
        return ""
    from piper_pp.kinematics import tcp_of

    class _FK:
        """pyAgxArm がない環境でも描けるよう、MDH で順運動学（SDK と同じ式）"""
        def __init__(self):
            from pyAgxArm.api.constants import ROBOT_MDH_PRESET
            self.mdh = ROBOT_MDH_PRESET["piper"]

        def fk(self, q):
            from scipy.spatial.transform import Rotation as Rot
            T = np.eye(4)
            for (d, a, al, off), qi in zip(self.mdh, q):
                ca, sa, ct, st = math.cos(al), math.sin(al), math.cos(qi + off), math.sin(qi + off)
                T = T @ np.array([[ct, -st, 0, a], [ca * st, ca * ct, -sa, -sa * d], [sa * st, sa * ct, ca, ca * d], [0, 0, 0, 1]])
            return list(T[:3, 3]) + list(Rot.from_matrix(T[:3, :3]).as_euler("xyz"))
    fk = _FK()
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    for R, c, n in ((TA, CA, "A"), (TB, CB, "B")):
        if not R:
            continue
        cyc = sorted(set(r["cycle"] for r in R), key=lambda v: int(v))[0]
        S = [r for r in R if r["cycle"] == cyc and r["segment"] in ("pre", "grasp", "lift", "transport", "place", "retreat")]
        if not S:
            continue
        Q = np.array([[num(r[f"j{i}"]) for i in range(1, 7)] for r in S])
        P = np.array([tcp_of(fk, q, CFG["tcp_z"])[0] for q in Q]) * 1000
        axs[0].plot(P[:, 0], P[:, 2], "-", color=c, lw=2, label=f"{n}（サイクル {cyc}）")
        axs[1].plot(P[:, 0], P[:, 1], "-", color=c, lw=2, label=n)
    axs[0].set_xlabel("x [mm]"); axs[0].set_ylabel("z [mm]"); axs[0].set_title("横から"); axs[0].grid(alpha=.3); axs[0].legend(); axs[0].set_aspect("equal", adjustable="datalim")
    axs[1].set_xlabel("x [mm]"); axs[1].set_ylabel("y [mm]"); axs[1].set_title("上から"); axs[1].grid(alpha=.3); axs[1].set_aspect("equal", adjustable="datalim")
    out = [f"<h2>6. 実際の手先の軌跡</h2><figure><img src='{fig_b64(fig)}'><figcaption>pre から retreat まで。実機の関節角（約 20 Hz）から順運動学で計算</figcaption></figure>"]
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.4), sharey=True)
    for ax, (R, c, n) in zip(axs, ((TA, CA, "A"), (TB, CB, "B"))):
        S = [r for r in R if r["segment"] == "transport"]
        if not S:
            ax.set_title(f"{n}: 記録なし"); continue
        cyc = S[0]["cycle"]; S = [r for r in S if r["cycle"] == cyc]
        t = col(S, "t_s")
        for i in range(1, 7):
            ax.plot(t, col(S, f"j{i}"), label=f"J{i}")
        ax.set_title(f"{n}: transport の関節角", color=c); ax.set_xlabel("時間 [s]"); ax.grid(alpha=.3)
    axs[0].set_ylabel("角度 [°]"); axs[1].legend(fontsize=7, ncol=3)
    out.append(f"<figure><img src='{fig_b64(fig)}'><figcaption>運搬区間の関節角の時間変化（滑らかさ・同時に動く関節の違い）</figcaption></figure>")
    return "".join(out)


def sec_notes():
    p = data_path("report_notes.md")
    if not p.exists():
        return "<h2>7. 所見</h2><p class='na'>data/report_notes.md に所見を書くと、ここに入る</p>"
    out = ["<h2>7. 所見</h2>"]
    for block in p.read_text(encoding="utf-8").split("\n## "):
        lines = block.strip().lstrip("#").strip().splitlines()
        if not lines:
            continue
        out.append(f"<h3>{html.escape(lines[0])}</h3>")
        body = "\n".join(lines[1:]).strip()
        for para in body.split("\n\n"):
            items = [ln[2:] for ln in para.splitlines() if ln.startswith("- ")]
            if items and len(items) == len(para.splitlines()):
                out.append("<ul>" + "".join(f"<li>{html.escape(i)}</li>" for i in items) + "</ul>")
            elif para.strip():
                out.append(f"<p>{html.escape(para)}</p>")
    return "".join(out)


CSS = """
body{margin:0;background:#fff;color:#1b1f24;font-family:'Noto Sans CJK JP','Noto Sans JP','Hiragino Sans','Yu Gothic',sans-serif;font-size:10.5pt;line-height:1.65}
main{max-width:190mm;margin:0 auto;padding:14mm 0 20mm}
@media screen and (max-width:760px){main{padding:16px}}
h1{font-size:19pt;margin:0 0 2mm}h2{font-size:13pt;margin:8mm 0 3mm;padding-bottom:1.5mm;border-bottom:2px solid #1f5fa8;break-after:avoid}
h3{font-size:11pt;margin:5mm 0 2mm;break-after:avoid}
.meta{color:#5c636b;font-size:9pt;margin-bottom:4mm}
table{width:100%;border-collapse:collapse;font-size:8.8pt;margin:2mm 0 3mm;display:block;overflow-x:auto}
th,td{border:1px solid #d5dae0;padding:1.2mm 1.8mm;text-align:left;vertical-align:top}th{background:#f3f5f7}
tr{break-inside:avoid}figure{margin:3mm 0;break-inside:avoid}img{max-width:100%}
figcaption{font-size:8.6pt;color:#5c636b}.note{font-size:8.8pt;color:#5c636b}.na{color:#9a5b00}
@page{size:A4;margin:14mm 12mm 16mm}
"""


def main():
    title = sys.argv[sys.argv.index("--title") + 1] if "--title" in sys.argv else "PiPER ボトル往復：A（Python）と B（ROS2＋MoveIt＋YOLO）の比較"
    body = [f"<h1>{html.escape(title)}</h1><div class='meta'>作成 {time.strftime('%Y-%m-%d %H:%M')} ／ piper-bottle-pp ／ データ: data/</div>",
            sec_conditions(), sec_p1(), sec_vision(), sec_cycles(), sec_segments(), sec_traj(), sec_notes()]
    doc = f"<!doctype html><html lang='ja'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>{html.escape(title)}</title><style>{CSS}</style></head><body><main>{''.join(body)}</main></body></html>"
    out = ROOT / "docs" / "report_A_vs_B.html"
    out.write_text(doc, encoding="utf-8")
    print(f"レポート: {out}")
    if "--pdf" in sys.argv:
        try:
            from playwright.sync_api import sync_playwright
            with sync_playwright() as p:
                b = p.chromium.launch(); pg = b.new_page(); pg.goto(out.as_uri()); pg.wait_for_timeout(500)
                pg.pdf(path=str(out.with_suffix(".pdf")), format="A4", print_background=True,
                       margin=dict(top="14mm", bottom="16mm", left="12mm", right="12mm"))
                b.close()
            print(f"PDF: {out.with_suffix('.pdf')}")
        except Exception as e:
            print(f"PDF は作れませんでした（{e}）。HTML をブラウザで開いて印刷 → PDF に保存してください")


if __name__ == "__main__":
    main()
