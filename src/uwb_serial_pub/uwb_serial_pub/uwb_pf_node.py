#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

# Import the particle filter class from another file in the same package
from .particle_filter import ParticleFilter


class UwbParticleFilterNode(Node):
    """
    ROS2 node that:
    - Subscribes to 4 UWB range topics (Float32)
    - Runs a ParticleFilter instance (defined in particle_filter.py)
    - Publishes the estimated pose as PoseStamped
    - Publishes a Marker (sphere) in RViz at the estimated position
    """

    def __init__(self):
        super().__init__('uwb_pf_node')

        # ---------------- Basic filter configuration ----------------
        # You can later turn these into ROS parameters, for now hard-coded
        N = 20000          # number of particles
        dt = 0.5         # initial dt guess, replaced by measured intervals
        sigma_pos = 0.3  # motion noise in position
        sigma_v = 0.1    # motion noise in velocity
        sigma_theta = 0.05  # motion noise in heading
        gamma = 0.2     # likelihood scale (meters)

        # Anchor positions in the robot base frame [m]
        anchors = [
            (-0.225,  0.195),   # anchor 1
            ( 0.225,  0.195),   # anchor 2
            ( -0.225, -0.195),   # anchor 3
            ( 0.225, -0.195),   # anchor 4
        ]

        # Create the particle filter object (class is defined in particle_filter.py)
        self.pf = ParticleFilter(
            N=N,
            dt=dt,
            sigma_pos=sigma_pos,
            sigma_v=sigma_v,
            sigma_theta=sigma_theta,
            cauchy_gamma=gamma,
            anchors=anchors,
        )

        # Initialize particles in a broad area around the robot
        px_range = (-5.0, 5.0)
        py_range = (-5.0, 5.0)
        v_range = (0.0, 0.1)
        theta_range = (-math.pi, math.pi)
        self.pf.initialize(px_range, py_range, v_range, theta_range)

        # ---------------- Range topics ----------------
        # Update these to the actual topic names in your system
        self.range_topics = [
            'uwb/anchor1/distance_m',
            'uwb/anchor2/distance_m',
            'uwb/anchor3/distance_m',
            'uwb/anchor4/distance_m',
        ]

        # Store the last range measurement from each anchor (initially None)
        self.latest_ranges = [None] * len(self.range_topics)
        self.latest_times = [None] * len(self.range_topics)

        # One subscription per anchor
        self.subs = []
        for idx, topic in enumerate(self.range_topics):
            sub = self.create_subscription(
                Float32,
                topic,
                # Use lambda to capture the anchor index
                lambda msg, i=idx: self.range_callback(msg, i),
                10,
            )
            self.subs.append(sub)

        # ---------------- Publishers for RViz ----------------
        # PoseStamped – can be used by other nodes
        self.pose_pub = self.create_publisher(PoseStamped, 'uwb/pf_pose', 10)

        # Marker – sphere in RViz at the estimated position
        self.marker_pub = self.create_publisher(Marker, 'uwb/pf_marker', 10)

        # Frame in which we publish (set this as fixed frame in RViz)
        self.frame_id = 'map'
        self.last_pf_time = None

        self.get_logger().info('UWB Particle Filter node started.')

    # -------------------------------------------------------
    #      Callback for each incoming range measurement
    # -------------------------------------------------------
    def range_callback(self, msg: Float32, idx: int):

        # Current timestamp of this measurement
        msg_time = self.get_clock().now()

        # Store the latest measurement and timestamp
        self.latest_ranges[idx] = float(msg.data)
        self.latest_times[idx] = msg_time

        # We must have at least one measurement from each anchor
        if any(r is None for r in self.latest_ranges):
            return

        # Convert list -> numpy array
        ranges = np.array(self.latest_ranges, dtype=float)

        # Use a fixed time step for the PF (e.g., assuming ~10 Hz sensor rates)
        dt = 0.1

        # Run the particle filter step (prediction + update + resampling)
        self.pf.step(ranges, dt=dt)

        # Optionally: do NOT clear the buffers.
        # Keeping the latest measurements makes the filter run more smoothly.
        #
        # If you do want to clear values every cycle, uncomment this:
        #
        # for i in range(len(self.latest_ranges)):
        #     self.latest_ranges[i] = None
        #     self.latest_times[i] = None

        # Estimate the state from the particle set (weighted mean)
        state = self.pf.estimate()
        px, py, v, theta = state

        # Publish estimated pose to ROS
        self.publish_pose(px, py, theta)

        # Publish a small sphere marker in RViz
        self.publish_marker(px, py)

        # Debug print to terminal
        self.get_logger().info(
            f'PF estimate: x={px:.2f} m, y={py:.2f} m, v={v:.2f} m/s, '
            f'theta={math.degrees(theta):.1f} deg'
        )


    # -------------------------------------------------------
    #            Publish PoseStamped (position + heading)
    # -------------------------------------------------------
    def publish_pose(self, px: float, py: float, theta: float):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(px)
        msg.pose.position.y = float(py)
        msg.pose.position.z = 0.0

        # Convert yaw (theta) to quaternion around z-axis
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(theta * 0.5)
        msg.pose.orientation.w = math.cos(theta * 0.5)

        self.pose_pub.publish(msg)

    # -------------------------------------------------------
    #            Publish a Marker sphere in RViz
    # -------------------------------------------------------
    def publish_marker(self, px: float, py: float):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'uwb_pf'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(px)
        marker.pose.position.y = float(py)
        marker.pose.position.z = 0.0

        # No orientation for a pure point, keep identity quaternion
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Sphere size in RViz [m]
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        # Sphere color (cyan-like)
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.9  # alpha

        # 0 -> marker persists until overwritten
        marker.lifetime.sec = 0

        self.marker_pub.publish(marker)


# -------------------------------------------------------
#                    main – node runner
# -------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = UwbParticleFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
