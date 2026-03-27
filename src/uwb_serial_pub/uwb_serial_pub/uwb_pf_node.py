#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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

        # ---------------- Parameters ----------------
        self.declare_parameter('verbose', True)
        self.declare_parameter('use_sensor_qos', True)
        self.declare_parameter('min_required_anchors', 3)
        self.declare_parameter('n_particles', 20000)
        self.declare_parameter('init_dt', 0.5)
        self.declare_parameter('pf_dt', 0.1)
        self.declare_parameter('sigma', 0.3)
        self.declare_parameter('neff_threshhold_ratio', 0.5)
        self.declare_parameter('sigma_position', 0.02)
        self.declare_parameter('sigma_theta', 0.05)
        self.declare_parameter('sigma_velocity', 0.1)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pose_topic', '/uwb/pf_pose')
        self.declare_parameter('marker_topic', '/uwb/pf_marker')
        self.declare_parameter('anchor_topics', [
            '/uwb/anchor1/distance_m',
            '/uwb/anchor2/distance_m',
            '/uwb/anchor3/distance_m',
            '/uwb/anchor4/distance_m',
        ])
        self.declare_parameter('anchor_positions_xy', [
            -0.225, 0.195,
             0.225, 0.195,
            -0.225, -0.195,
             0.225, -0.195,
        ])
        self.declare_parameter('init_x_range', [-5.0, 5.0])
        self.declare_parameter('init_y_range', [-5.0, 5.0])
        self.declare_parameter('init_theta_range', [-math.pi, math.pi])
        self.declare_parameter('init_v_range', [0.0, 0.1])
        self.declare_parameter('wait_log_period_sec', 2.0)

        self.verbose = bool(self.get_parameter('verbose').value)
        use_sensor_qos = bool(self.get_parameter('use_sensor_qos').value)
        self.min_required_anchors = int(self.get_parameter('min_required_anchors').value)
        n_particles = int(self.get_parameter('n_particles').value)
        init_dt = float(self.get_parameter('init_dt').value)
        self.pf_dt = float(self.get_parameter('pf_dt').value)
        sigma = float(self.get_parameter('sigma').value)
        neff_threshhold_ratio = float(self.get_parameter('neff_threshhold_ratio').value)
        sigma_position = float(self.get_parameter('sigma_position').value)
        sigma_theta = float(self.get_parameter('sigma_theta').value)
        sigma_velocity = float(self.get_parameter('sigma_velocity').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        anchor_topics = list(self.get_parameter('anchor_topics').value)
        if len(anchor_topics) != 4:
            raise ValueError(
                f'anchor_topics must contain 4 topics, got {len(anchor_topics)}'
            )
        self.range_topics = [str(topic) for topic in anchor_topics]

        anchor_xy = list(self.get_parameter('anchor_positions_xy').value)
        if len(anchor_xy) != 8:
            raise ValueError(
                f'anchor_positions_xy must contain 8 values, got {len(anchor_xy)}'
            )
        anchors = [
            (float(anchor_xy[0]), float(anchor_xy[1])),
            (float(anchor_xy[2]), float(anchor_xy[3])),
            (float(anchor_xy[4]), float(anchor_xy[5])),
            (float(anchor_xy[6]), float(anchor_xy[7])),
        ]
        self.anchor_positions = anchors

        if not (1 <= self.min_required_anchors <= len(self.range_topics)):
            raise ValueError(
                f'min_required_anchors must be in [1, {len(self.range_topics)}], '
                f'got {self.min_required_anchors}'
            )

        init_x_range = tuple(float(v) for v in self.get_parameter('init_x_range').value)
        init_y_range = tuple(float(v) for v in self.get_parameter('init_y_range').value)
        init_theta_range = tuple(float(v) for v in self.get_parameter('init_theta_range').value)
        init_v_range = tuple(float(v) for v in self.get_parameter('init_v_range').value)
        self.wait_log_period_sec = float(self.get_parameter('wait_log_period_sec').value)

        # Create the particle filter object (class is defined in particle_filter.py)
        self.pf = ParticleFilter(
            N=n_particles,
            dt=init_dt,
            sigma=sigma,
            anchors=anchors,
            neff_threshhold_ratio=neff_threshhold_ratio,
            sigma_position=sigma_position,
            sigma_theta=sigma_theta,
            sigma_velocity=sigma_velocity,
        )

        # Initialize particles in a broad area around the robot
        self.pf.initialize(init_x_range, init_y_range, init_theta_range, init_v_range)

        # Store the last range measurement from each anchor (initially None)
        self.latest_ranges = [None] * len(self.range_topics)
        self.latest_times = [None] * len(self.range_topics)

        # One subscription per anchor
        self.subs = []
        sub_qos = qos_profile_sensor_data if use_sensor_qos else 10
        for idx, topic in enumerate(self.range_topics):
            sub = self.create_subscription(
                Float32,
                topic,
                # Use lambda to capture the anchor index
                lambda msg, i=idx: self.range_callback(msg, i),
                sub_qos,
            )
            self.subs.append(sub)

        # ---------------- Publishers for RViz ----------------
        # PoseStamped – can be used by other nodes
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        # Marker – sphere in RViz at the estimated position
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.last_pf_time = None
        self.last_wait_log_time = 0.0

        self.get_logger().info(
            'UWB Particle Filter node started. '
            f'verbose={self.verbose}, use_sensor_qos={use_sensor_qos}, '
            f'min_required_anchors={self.min_required_anchors}, '
            f'n_particles={n_particles}, pf_dt={self.pf_dt}, sigma={sigma}, '
            f'neff_threshhold_ratio={neff_threshhold_ratio}, '
            f'sigma_position={sigma_position}, sigma_theta={sigma_theta}, '
            f'sigma_velocity={sigma_velocity}'
        )

    # -------------------------------------------------------
    #      Callback for each incoming range measurement
    # -------------------------------------------------------
    def range_callback(self, msg: Float32, idx: int):

        # Current timestamp of this measurement
        msg_time = self.get_clock().now()

        # Store the latest measurement and timestamp
        self.latest_ranges[idx] = float(msg.data)
        self.latest_times[idx] = msg_time

        available_idx = [i for i, r in enumerate(self.latest_ranges) if r is not None]

        # Require a minimum number of available anchors (default: 3).
        if len(available_idx) < self.min_required_anchors:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_wait_log_time >= self.wait_log_period_sec:
                missing = [
                    i + 1 for i, r in enumerate(self.latest_ranges) if r is None
                ]
                self.get_logger().warn(
                    f'Waiting for anchor ranges, have {len(available_idx)}/'
                    f'{self.min_required_anchors} required, missing anchors: {missing}'
                )
                self.last_wait_log_time = now_sec
            return

        # Build measurement vector from currently available anchors only.
        ranges = np.array([self.latest_ranges[i] for i in available_idx], dtype=float)
        active_anchors = [self.anchor_positions[i] for i in available_idx]

        # Run the particle filter step (prediction + update + resampling)
        self.pf.step(ranges, dt=self.pf_dt, anchors=active_anchors)

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
        px, py, theta, v = state

        # Publish estimated pose to ROS
        self.publish_pose(px, py, theta)

        # Publish a small sphere marker in RViz
        self.publish_marker(px, py)

        # Debug print to terminal
        if self.verbose:
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
