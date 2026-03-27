#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class UwbBayesDirectionNode(Node):
    """
    ROS2 node that:
    - Subscribes to 4 UWB range topics
    - Runs a simple 2D Bayesian grid filter to estimate the tag position
    - Publishes an RViz arrow (Marker) that points from the robot to the tag
    """

    def __init__(self):
        super().__init__('uwb_bayes_direction')

        # === Parameters (you can turn these into ROS parameters if you want) ===

        # Anchor positions in robot base frame [m]
        # Example: small rectangle around robot center
        # a1: front-left, a2: front-right, a3: rear-right, a4: rear-left
        # Anchor positions in robot base frame [m]
        self.anchors = np.array([
        [-0.225,  0.195],   # anchor 1
        [ 0.225,  0.195],   # anchor 2
        [-0.225, -0.195],   # anchor 3
        [ 0.225, -0.195],   # anchor 4
        ], dtype=float)


        # UWB noise standard deviation (assumed Gaussian) [m]
        self.sigma = 0.10
        # Small diffusion keeps the belief from collapsing to one cell forever
        self.process_noise = 0.0
        self.diffusion_alpha = 0.05

        # Grid limits [m] around the robot
        self.x_min = -3.0
        self.x_max =  3.0
        self.y_min = -3.0
        self.y_max =  3.0
        self.res   =  0.10   # grid resolution [m] (coarser grid → faster)

        # Frame ID for RViz (robot frame)
        self.frame_id = 'map'

        # Range topics (you can change to your actual topics)
        range_topics = [
            'uwb/anchor1/distance_m',
            'uwb/anchor2/distance_m',
            'uwb/anchor3/distance_m',
            'uwb/anchor4/distance_m',
        ]

        # === Build 2D grid and initialize belief ===
        self.x_vals = np.arange(self.x_min, self.x_max + 1e-6, self.res)
        self.y_vals = np.arange(self.y_min, self.y_max + 1e-6, self.res)
        self.X, self.Y = np.meshgrid(self.x_vals, self.y_vals)
        self.uniform_bel = np.ones_like(self.X, dtype=float)
        self.uniform_bel /= self.uniform_bel.sum()
        self.bel = self.uniform_bel.copy()  # uniform prior

        # Save latest 4 ranges (None until received)
        self.ranges = [None, None, None, None]

        # Subscribers
        self.subs = []
        for i, topic in enumerate(range_topics):
            sub = self.create_subscription(
                Float32,
                topic,
                lambda msg, idx=i: self.range_callback(msg, idx),
                10
            )
            self.subs.append(sub)

        # Publisher for RViz Marker
        self.marker_pub = self.create_publisher(Marker, 'uwb_direction_marker', 10)

        # Timer to re-publish the last estimate at fixed rate (for RViz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Last estimated position and direction
        self.last_pos = (0.0, 0.0)
        self.last_vec = (0.0, 0.0)
        self.last_theta = 0.0

        self.get_logger().info('UWB Bayesian direction node started')

    # ======================================================================
    # ROS Callbacks
    # ======================================================================

    def range_callback(self, msg: Float32, idx: int):
        """Called whenever a new range from anchor idx arrives."""
        self.ranges[idx] = msg.data

        # Check if we have all 4 ranges
        if all(r is not None for r in self.ranges):
            z = np.array(self.ranges, dtype=float)
            self.process_ranges(z)

    def timer_callback(self):
        """Periodically publish the direction arrow to RViz."""
        self.publish_marker()

    # ======================================================================
    # Bayesian grid filter logic
    # ======================================================================

    def likelihood(self, z, x, y):
        """
        Compute p(z | x,y) assuming independent Gaussian noise on each anchor.
        z: np.array shape (4,) – measured ranges
        (x, y): scalar position for the tag
        """
        dx = x - self.anchors[:, 0]
        dy = y - self.anchors[:, 1]
        d_hat = np.sqrt(dx*dx + dy*dy)
        e = z - d_hat

        coef = 1.0 / (math.sqrt(2.0 * math.pi) * self.sigma)
        p = coef * np.exp(-0.5 * (e*e) / (self.sigma**2))

        # Product of the 4 anchors' likelihoods
        return float(np.prod(p))

    def bayes_update(self, bel, z):
        """Perform measurement update over the whole grid."""
        new_bel = np.zeros_like(bel)

        # NOTE: This is a naive double loop.
        # For real-time use you should:
        #  - make the grid smaller
        #  - or vectorize this using NumPy
        for i in range(self.X.shape[0]):
            for j in range(self.X.shape[1]):
                x = self.X[i, j]
                y = self.Y[i, j]
                p_z_given_xy = self.likelihood(z, x, y)
                new_bel[i, j] = p_z_given_xy * bel[i, j]

        # Normalize
        s = new_bel.sum()
        if s > 0.0:
            new_bel /= s
        else:
            # In case of numerical issues, keep old belief
            self.get_logger().warn('Belief sum is zero, skipping update')
            return bel

        return new_bel

    def estimate_position(self, bel):
        """Compute weighted average position (x_hat, y_hat)."""
        w = bel
        x_hat = float((self.X * w).sum())
        y_hat = float((self.Y * w).sum())
        return x_hat, y_hat

    def diffuse_belief(self, bel):
        """Blur the grid slightly so the peak can move as the tag moves."""
        if self.diffusion_alpha <= 0.0:
            return bel
        padded = np.pad(bel, 1, mode='edge')
        blurred = (
            padded[0:-2, 0:-2] + padded[0:-2, 1:-1] + padded[0:-2, 2:] +
            padded[1:-1, 0:-2] + padded[1:-1, 1:-1] + padded[1:-1, 2:] +
            padded[2:, 0:-2] + padded[2:, 1:-1] + padded[2:, 2:]
        ) / 9.0
        return (1.0 - self.diffusion_alpha) * bel + self.diffusion_alpha * blurred

    def apply_process_noise(self, bel):
        """Optional uniform mixing + diffusion to keep the estimate responsive."""
        bel = self.diffuse_belief(bel)
        if self.process_noise > 0.0:
            bel = (1.0 - self.process_noise) * bel + self.process_noise * self.uniform_bel
        return bel

    def compute_direction_vector(self, x_hat, y_hat):
        """Compute unit vector and heading angle from robot to tag."""
        r = math.sqrt(x_hat*x_hat + y_hat*y_hat)
        if r < 1e-6:
            return (0.0, 0.0), 0.0

        vx = x_hat / r
        vy = y_hat / r
        theta = math.atan2(y_hat, x_hat)
        return (vx, vy), theta

    def process_ranges(self, z):
        """
        Main pipeline for each new full set of ranges:
        - Bayes update
        - Estimate position
        - Compute direction vector
        """
        # 1) Prediction step: for now, no motion model → identity
        bel_pred = self.apply_process_noise(self.bel)

        # 2) Measurement update
        self.bel = self.bayes_update(bel_pred, z)

        # 3) Estimate position
        x_hat, y_hat = self.estimate_position(self.bel)

        # 4) Direction vector
        v, theta = self.compute_direction_vector(x_hat, y_hat)

        self.last_pos = (x_hat, y_hat)
        self.last_vec = v
        self.last_theta = theta

        self.get_logger().info(
            f"Tag pos: x={x_hat:.2f} m, y={y_hat:.2f} m | "
            f"dir=({v[0]:.2f}, {v[1]:.2f}), theta={math.degrees(theta):.1f} deg"
        )

    # ======================================================================
    # RViz marker
    # ======================================================================

    def publish_marker(self):
        """Publish an arrow from (0,0,0) to the estimated tag position."""
        x_hat, y_hat = self.last_pos

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'uwb_direction'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow goes from robot origin to estimated tag position
        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.0

        end = Point()
        end.x = float(x_hat)
        end.y = float(y_hat)
        end.z = 0.0

        marker.points = [start, end]

        # Arrow scale: x = length, y,z = width
        marker.scale.x = 0.05   # shaft diameter
        marker.scale.y = 0.10   # head width
        marker.scale.z = 0.10   # head height

        # Color (red arrow)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = UwbBayesDirectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
