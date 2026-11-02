#!/usr/bin/env python3
"""認識ノード（B）: サービスを呼ばれたら撮影し、台A・B のキャップ位置を返す

使い方（venv_pp で、ROS2 と agx_arm_ws を source した端末）:
  python ros2/vision_node.py                          # V1（YOLO で選んでから点群）
  python ros2/vision_node.py --ros-args -p use_yolo:=false   # V0（点群の形だけ、A と同じ）
呼び出し: ros2 service call /bottle_vision/locate std_srvs/srv/Trigger
  → message に JSON: {"A": {"cap": [x,y,z], "height": h, "n": 点数} or null, "B": ..., "t_s": 秒, "yolo_ms": ms, ...}

中身は A と同じ piper_pp.runlog.locate_stations。違いは、腕の姿勢を CAN から直接ではなく
/feedback/joint_states（実機の関節角）から順運動学で求めること。腕が止まっている時に呼ぶ。
"""
import json
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

import rosutil  # noqa: F401  (piper_pp を import できるようにする)
from rosutil import Feedback, offline_robot
from piper_pp.config import load_json, data_path
from piper_pp.runlog import locate_stations, FlangeFromJoints


class BottleVision(Node):
    def __init__(self):
        super().__init__("bottle_vision")
        self.declare_parameter("use_yolo", True)
        self.use_yolo = bool(self.get_parameter("use_yolo").value)
        self.fb = Feedback(self)
        self.fk = offline_robot()
        if self.use_yolo:
            from piper_pp import vision_yolo as VY
            self.get_logger().info("YOLO を読み込み中…")
            VY.load_model()
        self.create_service(Trigger, "~/locate", self.on_locate)
        self.pub = self.create_publisher(MarkerArray, "~/caps", 1)
        self.n_calls = 0
        self.get_logger().info(f"準備完了（{'V1: YOLO＋点群' if self.use_yolo else 'V0: 点群のみ'}）")

    def on_locate(self, req, res):
        q, _, _ = self.fb.get()
        if q is None:
            res.success = False
            res.message = json.dumps({"error": "feedback/joint_states が来ていない（agx_arm_ctrl を起動したか）"})
            return res
        st = load_json("stations.json")
        if not {"A", "B"} <= set(st):
            res.success = False
            res.message = json.dumps({"error": "data/stations.json に台A・B がない（A の 09 --setup を先に）"})
            return res
        robot = FlangeFromJoints(self.fk, lambda: q)
        try:
            loc = locate_stations(robot, st, self.use_yolo)
        except SystemExit as e:          # カメラが使えないなど
            res.success = False
            res.message = json.dumps({"error": str(e)})
            return res
        out = {s: (None if r is None else {"cap": r["cap"], "height": r["height"], "n": r["n"]}) for s, r in loc["found"].items()}
        out.update(vision="V1" if self.use_yolo else "V0", t_s=loc["t_s"], yolo_ms=loc["yolo_ms"],
                   n_boxes=len(loc["boxes"]), best_conf=max([b["conf"] for b in loc["boxes"]], default=None))
        res.success = True
        res.message = json.dumps(out)
        self.publish_markers(out)
        self.n_calls += 1
        if self.n_calls == 1 and loc["color"] is not None and self.use_yolo:
            self.save_image(loc)
        self.get_logger().info(f"locate: A={'有' if out['A'] else '無'} B={'有' if out['B'] else '無'} {loc['t_s']*1000:.0f} ms")
        return res

    def publish_markers(self, out):
        arr = MarkerArray()
        for i, s in enumerate(("A", "B")):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = self.get_clock().now().to_msg()
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
        self.pub.publish(arr)

    def save_image(self, loc):
        import cv2
        from piper_pp.camera import for_display
        vis = loc["color"].copy()
        for b in loc["boxes"]:
            x1, y1, x2, y2 = [int(v) for v in b["box"]]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 165, 255), 2)
        cv2.imwrite(str(data_path("vision_B_first.jpg")), for_display(vis))


def main():
    rclpy.init(args=sys.argv)
    node = BottleVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
