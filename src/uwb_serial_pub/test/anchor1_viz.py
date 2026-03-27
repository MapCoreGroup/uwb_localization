#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

class AnchorCircleViz(Node):
    def __init__(self):
        super().__init__('anchor_circle_viz')
        self.subscription = self.create_subscription(Float32, 'uwb/anchor1/distance_m', self.update_radius, 10)
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.anchor_position = [-0.225, 0.195, 0.0]
        self.current_radius = 0.5

    def update_radius(self, msg):
        self.current_radius = msg.data
        self.publish_circle()

    def publish_circle(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uwb_circle"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        points = []
        steps = 100
        for i in range(steps + 1):
            angle = 2 * math.pi * i / steps
            x = self.anchor_position[0] + self.current_radius * math.cos(angle)
            y = self.anchor_position[1] + self.current_radius * math.sin(angle)
            z = self.anchor_position[2]
            p = Point()
            p.x, p.y, p.z = x, y, z
            points.append(p)

        marker.points = points
        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = AnchorCircleViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
