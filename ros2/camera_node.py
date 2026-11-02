#!/usr/bin/env python3
"""カメラノード（B）: 深度の点群を常時配信し、呼ばれたらキャップ位置を返す

vision_node.py との違い: カメラを開きっぱなしにして
  ① 点群を /camera/depth/color/points に配信（MoveIt の Octomap が障害物として取り込む）
  ② サービス /bottle_vision/locate（vision_node.py と同じ名前・同じ JSON）
  ③ フランジ（link6）→ カメラの TF を配信（ハンドアイ校正 data/T_flange_cam.npy）
カメラは1プロセスしか開けないので、vision_node.py とは同時に起動しない。

使い方（venv_pp で、ROS2 と agx_arm_ws を source した端末）:
  python ros2/camera_node.py                                   # V1（YOLO）＋点群
  python ros2/camera_node.py --ros-args -p use_yolo:=false     # V0＋点群
  パラメータ: cloud_hz（既定 2.0）、voxel（間引きの格子 [m]、既定 0.01）
点群に入れるもの: 置き面より 30 mm 以上上（深度の誤差で台の天面が入らないように）、ロボットから 0.9 m 以内の点（台の天面は計画シーンの箱で表すので除く）
"""
import json
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

import rosutil  # noqa: F401
from rosutil import Feedback, offline_robot
from piper_pp.config import CFG, load_json, data_path
from piper_pp import camera, vision
from piper_pp.kinematics import flange_matrix
from piper_pp.runlog import locate_from_points

CAM_FRAME = "camera_color_optical_frame"


class CameraNode(Node):
    def __init__(self):
        super().__init__("bottle_vision")
        self.use_yolo = bool(self.declare_parameter("use_yolo", True).value)
        self.cloud_hz = float(self.declare_parameter("cloud_hz", 2.0).value)
        self.voxel = float(self.declare_parameter("voxel", 0.01).value)
        self.fb = Feedback(self)
        self.fk = offline_robot()
        self.X = vision.load_handeye()                      # フランジ → カラーカメラ
        if self.use_yolo:
            from piper_pp import vision_yolo as VY
            self.get_logger().info("YOLO を読み込み中…"); VY.load_model()
        self.depths = deque(maxlen=CFG["vision"]["n_frames"])   # (時刻, 深度[m])
        self.color = None
        self.param = None
        self.lock = threading.Lock()
        self.stop = False
        self.cam_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.cam_thread.start()
        self.pub_cloud = self.create_publisher(PointCloud2, "/camera/depth/color/points", 1)
        self.pub_mark = self.create_publisher(MarkerArray, "~/caps", 1)
        self.pub_img = self.create_publisher(Image, "~/image", 1)      # YOLO の枠を描いた画像（RViz の Image で表示）
        self.create_service(Trigger, "~/locate", self.on_locate)
        self.create_timer(1.0 / self.cloud_hz, self.publish_cloud)
        self.publish_static_tf()
        self.n_calls = 0
        self.get_logger().info(f"準備完了（{'V1: YOLO' if self.use_yolo else 'V0'}＋点群 {self.cloud_hz} Hz）")

    # ---------- カメラ ----------
    def capture_loop(self):
        pipe = camera.start(depth=True, color=True)
        with self.lock:
            self.param = pipe.get_camera_param()
        try:
            while not self.stop:
                fr = pipe.wait_for_frames(200)
                if fr is None:
                    continue
                d, c = fr.get_depth_frame(), fr.get_color_frame()
                with self.lock:
                    if d is not None:
                        self.depths.append((time.monotonic(), camera.depth_m(d)))
                    if c is not None:
                        self.color = camera.decode_color(c)
        finally:
            pipe.stop()

    def frames_after(self, t_req, timeout=3.0):
        """t_req より後に撮った深度を n_frames 枚そろえて返す（腕が止まってからの画像だけ使う）"""
        n = CFG["vision"]["n_frames"]; t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                fresh = [d for t, d in self.depths if t > t_req]
                if len(fresh) >= n:
                    return fresh[-n:], (None if self.color is None else self.color.copy()), self.param
            time.sleep(0.02)
        return None, None, None

    # ---------- TF ----------
    def publish_static_tf(self):
        self.tfb = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "link6", CAM_FRAME     # URDF の link6 = SDK のフランジ（0.1 mm 以内）
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = [float(v) for v in self.X[:3, 3]]
        q = Rot.from_matrix(self.X[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = [float(v) for v in q]
        self.tfb.sendTransform(t)

    def T_base_cam(self, q):
        return flange_matrix(self.fk.fk(list(q))) @ self.X

    # ---------- 点群 ----------
    def publish_cloud(self):
        q, _, _ = self.fb.get()
        with self.lock:
            if not self.depths or self.param is None:
                return
            depth = self.depths[-1][1]; param = self.param
        if q is None:
            return
        Pc = vision.depth_points([depth], param)                 # カメラ座標
        T = self.T_base_cam(q)
        Pb = Pc @ T[:3, :3].T + T[:3, 3]
        keep = (Pb[:, 2] > CFG["surface_z"] + 0.03) & (np.linalg.norm(Pb[:, :2], axis=1) < 0.9) & (Pb[:, 2] < 0.8)
        Pc = Pc[keep]
        if len(Pc) and self.voxel > 0:                           # 1 cm 格子で間引く
            _, idx = np.unique(np.floor(Pc / self.voxel).astype(np.int32), axis=0, return_index=True)
            Pc = Pc[idx]
        h = Header(); h.stamp = self.get_clock().now().to_msg(); h.frame_id = CAM_FRAME
        self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(h, Pc.astype(np.float32).tolist()))

    # ---------- キャップ位置 ----------
    def on_locate(self, req, res):
        t_req = time.monotonic()
        q, _, _ = self.fb.get()
        st = load_json("stations.json")
        if q is None or not {"A", "B"} <= set(st):
            res.success = False
            res.message = json.dumps({"error": "feedback/joint_states が来ていない、または stations.json に台A・B がない"})
            return res
        t0 = time.perf_counter()
        depths, color, param = self.frames_after(t_req)
        if depths is None:
            res.success = False; res.message = json.dumps({"error": "深度がそろわない（カメラを確認）"}); return res
        q2, _, _ = self.fb.get()
        if max(abs(a - b) for a, b in zip(q, q2)) > 0.002:
            self.get_logger().warn("撮影中に腕が動いた（約 0.1° 超）。位置がずれる可能性")
        T = self.T_base_cam(q2)
        Pc = vision.depth_points(depths, param)
        P = Pc @ T[:3, :3].T + T[:3, 3]
        loc = locate_from_points(P, T, color, param, st, self.use_yolo, t0=t0)
        out = {s: (None if r is None else {"cap": r["cap"], "height": r["height"], "n": r["n"]}) for s, r in loc["found"].items()}
        out.update(vision="V1" if self.use_yolo else "V0", t_s=loc["t_s"], yolo_ms=loc["yolo_ms"],
                   n_boxes=len(loc["boxes"]), best_conf=max([b["conf"] for b in loc["boxes"]], default=None))
        res.success = True; res.message = json.dumps(out)
        self.publish_markers(out)
        self.publish_image(color, loc["boxes"])
        self.n_calls += 1
        if self.n_calls == 1 and color is not None and self.use_yolo:
            import cv2
            vis = color.copy()
            for b in loc["boxes"]:
                x1, y1, x2, y2 = [int(v) for v in b["box"]]
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 165, 255), 2)
            cv2.imwrite(str(data_path("vision_B_first.jpg")), camera.for_display(vis))
        self.get_logger().info(f"locate: A={'有' if out['A'] else '無'} B={'有' if out['B'] else '無'} {loc['t_s']*1000:.0f} ms")
        return res

    def publish_image(self, color, boxes):
        if color is None:
            return
        import cv2
        vis = color.copy()
        for b in boxes:
            x1, y1, x2, y2 = [int(v) for v in b["box"]]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 165, 255), 2)
        vis = np.ascontiguousarray(camera.for_display(vis))
        m = Image(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = CAM_FRAME
        m.height, m.width = vis.shape[:2]; m.encoding = "bgr8"; m.step = vis.shape[1] * 3
        m.data = vis.tobytes()
        self.pub_img.publish(m)

    def publish_markers(self, out):
        arr = MarkerArray()
        for i, s in enumerate(("A", "B")):
            m = Marker(); m.header.frame_id = "base_link"; m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = "caps", i
            if out[s] is None:
                m.action = Marker.DELETE
            else:
                m.type, m.action = Marker.SPHERE, Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = out[s]["cap"]
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.03
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.3, 0.1, 0.9
            arr.markers.append(m)
        self.pub_mark.publish(arr)


def main():
    rclpy.init(args=sys.argv)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop = True
    time.sleep(0.3)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
