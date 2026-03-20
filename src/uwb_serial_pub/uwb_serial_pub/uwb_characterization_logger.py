#!/usr/bin/env python3
import os
import json
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32, String


class UwbCharacterizationLogger(Node):
    """
    Listens to UWB distance topic, publishes experiment metadata,
    collects N samples, and saves raw data + statistics to disk.
    """

    def __init__(self):
        super().__init__('uwb_characterization_logger')

        # =====================
        # Parameters
        # =====================
        self.declare_parameter('distance_topic', '/uwb/anchor1/distance_m')
        self.declare_parameter('ground_truth_m', 1.0)
        self.declare_parameter('condition', 'LOS')
        self.declare_parameter('num_samples', 1000)
        self.declare_parameter('output_dir', '~/uwb_characterization')
        self.declare_parameter('run_name', '')

        self.distance_topic = self.get_parameter('distance_topic').value
        self.ground_truth_m = float(self.get_parameter('ground_truth_m').value)
        self.condition = self.get_parameter('condition').value
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )
        self.run_name = self.get_parameter('run_name').value

        # =====================
        # QoS for experiment metadata (latched)
        # =====================
        meta_qos = QoSProfile(depth=1)
        meta_qos.reliability = ReliabilityPolicy.RELIABLE
        meta_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_gt = self.create_publisher(
            Float32, '/experiment/ground_truth_distance', meta_qos
        )
        self.pub_condition = self.create_publisher(
            String, '/experiment/condition', meta_qos
        )
        self.pub_status = self.create_publisher(
            String, '/experiment/status', 10
        )

        # =====================
        # Data storage
        # =====================
        self.samples = []
        self.sample_count = 0
        self.finished = False

        # =====================
        # Subscriber
        # =====================
        self.sub = self.create_subscription(
            Float32,
            self.distance_topic,
            self.distance_callback,
            50
        )

        # Publish metadata once (latched)
        self.publish_metadata()

        self.get_logger().info(
            f"UWB characterization started | "
            f"GT = {self.ground_truth_m:.2f} m | "
            f"Condition = {self.condition} | "
            f"N = {self.num_samples}"
        )

        self.pub_status.publish(String(data='RUNNING'))

    def publish_metadata(self):
        self.pub_gt.publish(Float32(data=self.ground_truth_m))
        self.pub_condition.publish(String(data=self.condition))

    def distance_callback(self, msg: Float32):
        if self.finished:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        measured = float(msg.data)
        error = measured - self.ground_truth_m

        self.samples.append({
            'index': self.sample_count,
            'ros_time_sec': now_sec,
            'measured_m': measured,
            'error_m': error
        })

        self.sample_count += 1

        if self.sample_count % 100 == 0:
            self.get_logger().info(
                f"Collected {self.sample_count}/{self.num_samples} samples"
            )

        if self.sample_count >= self.num_samples:
            self.finish_experiment()

    def finish_experiment(self):
        self.finished = True

        errors = [s['error_m'] for s in self.samples]
        measurements = [s['measured_m'] for s in self.samples]

        mean_error = sum(errors) / len(errors)
        variance = sum((e - mean_error) ** 2 for e in errors) / (len(errors) - 1)
        std_error = math.sqrt(variance)
        rmse = math.sqrt(sum(e ** 2 for e in errors) / len(errors))

        # Folder structure
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        if not self.run_name:
            run_name = f"{self.ground_truth_m:.2f}m_{self.condition}_{timestamp}"
            run_name = run_name.replace('.', 'p')
        else:
            run_name = self.run_name

        run_dir = os.path.join(self.output_dir, run_name)
        os.makedirs(run_dir, exist_ok=True)

        # =====================
        # Save CSV
        # =====================
        csv_path = os.path.join(run_dir, 'samples.csv')
        with open(csv_path, 'w') as f:
            f.write('index,ros_time_sec,measured_m,error_m\n')
            for s in self.samples:
                f.write(
                    f"{s['index']},"
                    f"{s['ros_time_sec']:.9f},"
                    f"{s['measured_m']:.6f},"
                    f"{s['error_m']:.6f}\n"
                )

        # =====================
        # Save summary JSON
        # =====================
        summary = {
            'distance_topic': self.distance_topic,
            'ground_truth_m': self.ground_truth_m,
            'condition': self.condition,
            'num_samples': len(self.samples),
            'mean_measured_m': sum(measurements) / len(measurements),
            'bias_mean_error_m': mean_error,
            'std_error_m': std_error,
            'rmse_m': rmse,
            'min_error_m': min(errors),
            'max_error_m': max(errors)
        }

        json_path = os.path.join(run_dir, 'summary.json')
        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info("Experiment completed successfully")
        self.get_logger().info(f"Results saved to: {run_dir}")

        self.pub_status.publish(String(data='DONE'))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UwbCharacterizationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
