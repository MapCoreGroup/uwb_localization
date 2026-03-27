#!/usr/bin/env python3
import math
import time
from functools import partial

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from geometry_msgs.msg import Point


class MultiAnchorCircles(Node):
    def __init__(self):
        super().__init__('multi_anchor_circles')

        self.anchors = {
            1: {
                "name": "anchor1",
                "topic": "/uwb/anchor1/distance_m",
                # Circle is drawn in the anchor frame, so the center is (0,0,0).
                "pos": (0.0, 0.0, 0.0),
                "frame_id": "anchor1",
                "color": (1.0, 0.0, 0.0, 1.0),  # red
            },
            2: {
                "name": "anchor2",
                "topic": "/uwb/anchor2/distance_m",
                "pos": (0.0, 0.0, 0.0),
                "frame_id": "anchor2",
                "color": (0.0, 1.0, 0.0, 1.0),  # green
            },
            3: {
                "name": "anchor3",
                "topic": "/uwb/anchor3/distance_m",
                "pos": (0.0, 0.0, 0.0),
                "frame_id": "anchor3",
                # Circle is drawn in the anchor frame, so center is (0,0,0).
                "color": (0.0, 0.0, 1.0, 1.0),  # yellow
            },
            4: {
                "name": "anchor4",
                "topic": "/uwb/anchor4/distance_m",
                "pos": (0.0, 0.0, 0.0),
                "frame_id": "anchor4",
                "color": (1.0, 1.0, 0.0, 1.0),  # blue
            },
        }

        self.state = {
            aid: {"radius": 0.5, "last_update": 0.0} #put at first default radius of 0.5 meter
            for aid in self.anchors
        }

        self.subs = []
        for aid, cfg in self.anchors.items():
            sub = self.create_subscription(
                Float32,
                cfg["topic"],
                partial(self._distance_cb, aid=aid),
                10
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to {cfg['topic']} for {cfg['name']}")

        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self._publish_all)

        self.line_thickness = 0.01
        self.circle_steps = 100
        self.stale_timeout_sec = 1.0

    def _distance_cb(self, msg: Float32, aid: int):
        self.state[aid]["radius"] = float(msg.data)
        self.state[aid]["last_update"] = time.time()

    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        for aid, cfg in self.anchors.items():
            if time.time() - self.state[aid]["last_update"] > self.stale_timeout_sec:
                continue

            marker = self._make_circle_marker(
                marker_id=aid,
                ns=f"uwb_{cfg['name']}",
                center=cfg["pos"],
                radius=self.state[aid]["radius"],
                rgba=cfg["color"],
                frame_id=cfg["frame_id"],
                stamp=now
            )
            self.pub.publish(marker)

    def _make_circle_marker(self, marker_id, ns, center, radius, rgba, frame_id, stamp):
        (cx, cy, cz) = center
        (r, g, b, a) = rgba

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_thickness
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        pts = []
        steps = self.circle_steps
        for i in range(steps + 1):
            angle = 2.0 * math.pi * i / steps
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            p = Point()
            p.x, p.y, p.z = x, y, cz
            pts.append(p)
        marker.points = pts

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = MultiAnchorCircles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
