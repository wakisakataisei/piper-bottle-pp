"""手首カメラのハンドアイ校正（eye-in-hand）→ data/T_flange_cam.npy
使い方: python scripts/05_handeye.py <マーカー一辺[mm]> > /dev/null
        （腕の操作は別ターミナルの 02_jog.py で行う。このスクリプトは腕に指令を送らない）
  c: サンプル取得（マーカー検出中、腕が静止しているときのみ）  u: 直前を取り消し  q: 計算して終了
目安: 10〜15 姿勢。位置だけでなく手首の向きを ±20° 程度ずつ変える。ばらつき平均 3 mm 以下なら良好
"""
import sys
import time
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot
import _bootstrap  # noqa: F401
from piper_pp.config import CFG, data_path, save_json
from piper_pp.kinematics import flange_matrix
from piper_pp.handeye import solve_handeye

if len(sys.argv) < 2:
    sys.exit(__doc__)
from piper_pp import camera  # noqa: E402
from piper_pp.robot import connect  # noqa: E402

MARKER = float(sys.argv[1]) / 1000.0
TCP_Z = CFG["tcp_z"]
out = sys.stderr

pipe = camera.start(depth=False, color=True)
p = pipe.get_camera_param()
ri, rd = p.rgb_intrinsic, p.rgb_distortion
K = np.array([[ri.fx, 0, ri.cx], [0, ri.fy, ri.cy], [0, 0, 1]], float)
dist = np.array([rd.k1, rd.k2, rd.p1, rd.p2, rd.k3], float)
print(f"カラー内部パラメータ: fx={ri.fx:.1f} fy={ri.fy:.1f} cx={ri.cx:.1f} cy={ri.cy:.1f} ({ri.width}x{ri.height})", file=out)
if ri.fx <= 0:
    sys.exit("内部パラメータが取得できませんでした（fx<=0）")

robot = connect()


def flange_T():
    f = robot.get_flange_pose().msg
    return flange_matrix(f), f


# 姿勢角の解釈を自己検証（フランジ姿勢×[0,0,TCP_Z] が SDK の TCP と一致するか）
robot.set_tcp_offset([0, 0, TCP_Z, 0, 0, 0])
T, _ = flange_T()
err = np.linalg.norm(T[:3, :3] @ [0, 0, TCP_Z] + T[:3, 3] - np.array(robot.get_tcp_pose().msg[:3])) * 1000
print(f"姿勢角の解釈チェック: 計算TCPとSDKのTCPの差 {err:.1f} mm（1 mm 未満なら OK）", file=out)
if err > 1.0:
    sys.exit("オイラー角の順序が想定と違います（SDK の版を確認）")

det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
h = MARKER / 2
obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)

samples = []
print("別ターミナルの 02_jog.py で姿勢を変え、静止してから c キー。10〜15回集めて q キー", file=out)
while True:
    f = pipe.wait_for_frames(100)
    c = f.get_color_frame() if f else None
    if c is None:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        continue
    img = camera.decode_color(c)
    corners, ids, _ = det.detectMarkers(img)
    pose = None
    if ids is not None and 0 in ids.flatten():
        cc = corners[list(ids.flatten()).index(0)][0]
        ok, rv, tv = cv2.solvePnP(obj, cc, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            pose = (rv, tv)
            cv2.drawFrameAxes(img, K, dist, rv, tv, MARKER)
            cv2.aruco.drawDetectedMarkers(img, [cc[None]])
    show = camera.for_display(img)
    msg = f"samples {len(samples)}  " + (f"marker {float(np.linalg.norm(pose[1]))*1000:.0f} mm" if pose else "marker NOT found")
    cv2.putText(show, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if pose else (0, 0, 255), 2)
    cv2.imshow("handeye  c=capture u=undo q=solve", show)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    if k == ord("u") and samples:
        samples.pop(); print(f"取り消し → {len(samples)} 件", file=out)
    if k == ord("c"):
        if pose is None:
            print("マーカー未検出のため取得しません", file=out); continue
        T1, f1 = flange_T(); time.sleep(0.2); T2, _ = flange_T()
        if np.linalg.norm(T1[:3, 3] - T2[:3, 3]) > 0.0005:
            print("腕が動いています。止まってから取得してください", file=out); continue
        samples.append({"flange": list(map(float, f1)), "rvec": pose[0].ravel().tolist(), "tvec": pose[1].ravel().tolist()})
        print(f"取得 {len(samples)}: マーカー距離 {np.linalg.norm(pose[1])*1000:.0f} mm  フランジ {np.round(f1[:3], 3)}", file=out)
pipe.stop(); cv2.destroyAllWindows(); robot.disconnect()

save_json("handeye_samples.json", {"marker_m": MARKER, "K": K.tolist(), "dist": dist.tolist(), "samples": samples})
if len(samples) < 6:
    sys.exit(f"サンプルが {len(samples)} 件で不足（6件以上、推奨10件以上）")


def T_of(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


Tg = [flange_matrix(s["flange"]) for s in samples]
Tc = [T_of(cv2.Rodrigues(np.array(s["rvec"]))[0], s["tvec"]) for s in samples]
X, P, spread = solve_handeye(Tg, Tc)
pm = P.mean(0)
np.save(data_path("T_flange_cam.npy"), X)
print(f"\nカメラ位置（フランジ基準）: {np.round(X[:3,3]*1000,1)} mm", file=out)
print(f"マーカー位置のばらつき: 平均 {spread.mean():.1f} mm / 最大 {spread.max():.1f} mm（各サンプル: {np.round(spread,1)}）", file=out)
print(f"マーカー中心のロボット座標（推定）: x={pm[0]*1000:.1f} y={pm[1]*1000:.1f} z={pm[2]*1000:.1f} mm", file=out)
print("→ メジャーで測ったマーカー位置と数 mm 以内で一致するか確認。data/T_flange_cam.npy に保存", file=out)
