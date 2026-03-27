#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RangeStatsNode(Node):
    def __init__(self):
        super().__init__('range_stats_node')

        # === Parameters ===
        # True distance between anchor and tag [meters]
        self.declare_parameter('true_distance', 9.0)

        # Number of samples to collect before computing statistics
        self.declare_parameter('num_samples', 1000)

        # Topic name from which we read the measured ranges
        self.declare_parameter('input_topic', 'uwb/anchor1/distance_m')

        # Read parameter values
        self.true_distance = float(self.get_parameter('true_distance').value)
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.input_topic = self.get_parameter('input_topic').value

        self.get_logger().info(
            f"Starting RangeStatsNode: true_distance={self.true_distance} [m], "
            f"num_samples={self.num_samples}, input_topic='{self.input_topic}'"
        )

        # List of collected samples
        self.samples = []

        # Publisher for the standard deviation sigma(d)
        self.sigma_pub = self.create_publisher(Float32, 'uwb/range_sigma', 10)

        # Subscriber for the range measurements
        self.sub = self.create_subscription(
            Float32,
            self.input_topic,
            self.range_callback,
            10
        )

    def range_callback(self, msg: Float32):
        """Called every time a new range measurement is received."""
        r = float(msg.data)
        self.samples.append(r)

        n = len(self.samples)
        if n < 2:
            # Need at least 2 samples to compute standard deviation
            return

        if n >= self.num_samples:
            # Compute statistics over the collected window
            mean_r = sum(self.samples) / n
            var = sum((x - mean_r) ** 2 for x in self.samples) / (n - 1)
            sigma = math.sqrt(var)

            bias = mean_r - self.true_distance

            # Print stats to the log
            self.get_logger().info(
                f"Computed stats over {n} samples:\n"
                f"  true_distance = {self.true_distance:.4f} m\n"
                f"  mean(measured) = {mean_r:.4f} m\n"
                f"  sigma = {sigma:.4f} m\n"
                f"  bias  = {bias:.4f} m"
            )

            # Publish sigma to a topic
            sigma_msg = Float32()
            sigma_msg.data = float(sigma)
            self.sigma_pub.publish(sigma_msg)

            # Clear samples for the next batch
            self.samples.clear()
            # If you prefer a sliding window instead of reset,
            # replace this with logic that pops old samples.


def main(args=None):
    rclpy.init(args=args)
    node = RangeStatsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
