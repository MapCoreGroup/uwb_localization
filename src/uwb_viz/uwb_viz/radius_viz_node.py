#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import yaml, os
from math import isnan

class RangeMarkerNode(Node):
    def __init__(self):
        super().__init__('radius_viz')

        # Load configuration
        self.declare_parameter('config_path', '')
        cfg_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not cfg_path or not os.path.exists(cfg_path):
            self.get_logger().fatal(f"Config file not found: {cfg_path}")
            raise FileNotFoundError(cfg_path)

        with open(cfg_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.fixed_frame = self.cfg.get('fixed_frame', 'map')
        self.default_height = float(self.cfg.get('default_height', 0.02))
        self.default_color = self.cfg.get('default_color', [0.0, 0.6, 1.0, 0.35])
        self.anchors = self.cfg.get('anchors', [])

        if not self.anchors:
            self.get_logger().error("No anchors defined in YAML file!")
            return

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, 'uwb_radius_markers', 10)

        # Subscriptions for each anchor
        self.subs = []
        for idx, anchor in enumerate(self.anchors):
            topic = anchor['topic']
            sub = self.create_subscription(Float32, topic,
                                           lambda msg, i=idx: self.range_callback(i, msg),
                                           10)
            self.subs.append(sub)
            anchor['_last_range'] = None

        self.get_logger().info(f"Loaded {len(self.anchors)} anchors from {cfg_path}")
        self.timer = self.create_timer(0.3, self.republish_markers)

    def range_callback(self, anchor_idx, msg: Float32):
        r = float(msg.data)
        if r <= 0.0 or isnan(r):
            return
        self.anchors[anchor_idx]['_last_range'] = r
        self.publish_marker(anchor_idx, r)

    def republish_markers(self):
        for i, a in enumerate(self.anchors):
            r = a.get('_last_range', None)
            if r:
                self.publish_marker(i, r, transient=True)

    def publish_marker(self, anchor_idx, radius, transient=False):
        a = self.anchors[anchor_idx]
        name = a.get('name', f'anchor_{anchor_idx+1}')
        xyz = a.get('xyz', [0.0, 0.0, 0.0])
        color = a.get('color', self.default_color)
        r_c, g_c, b_c, alpha = color

        # Disk marker (cylinder)
        m = Marker()
        m.header.frame_id = self.fixed_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f"{name}_radius"
        m.id = anchor_idx * 2 + 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = xyz[0]
        m.pose.position.y = xyz[1]
        m.pose.position.z = xyz[2]
        m.pose.orientation.w = 1.0
        m.scale.x = 2.0 * radius
        m.scale.y = 2.0 * radius
        m.scale.z = self.default_height
        m.color.r, m.color.g, m.color.b, m.color.a = r_c, g_c, b_c, alpha
        m.lifetime.sec = 1
        self.marker_pub.publish(m)

        # Text label above the anchor
        t = Marker()
        t.header.frame_id = self.fixed_frame
        t.header.stamp = m.header.stamp
        t.ns = f"{name}_label"
        t.id = anchor_idx * 2 + 2
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = xyz[0]
        t.pose.position.y = xyz[1]
        t.pose.position.z = xyz[2] + 0.2
        t.pose.orientation.w = 1.0
        t.scale.z = 0.15
        t.color.r = 1.0
        t.color.g = 1.0
        t.color.b = 1.0
        t.color.a = 0.9
        t.text = f"{name}: {radius:.2f} m"
        t.lifetime.sec = 1
        self.marker_pub.publish(t)

def main():
    rclpy.init()
    node = RangeMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
