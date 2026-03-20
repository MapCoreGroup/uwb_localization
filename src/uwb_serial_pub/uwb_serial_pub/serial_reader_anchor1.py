#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
import re

class Anchor1Node(Node):
    def __init__(self):
        super().__init__('anchor1')

        # Parameters (use your existing symlink)
        self.declare_parameter('port', '/dev/Anchor1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Anchor1 connected on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            raise

        # Publishers
        self.pub_raw = self.create_publisher(String, '/uwb/anchor1/raw', 10)
        self.pub_dist = self.create_publisher(Float32, '/uwb/anchor1/distance_m', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.read_serial)  # 20 Hz

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                # Publish raw text
                self.pub_raw.publish(String(data=line))

                # Try to extract numeric distance from the line.
                # Expected format: "distance = <number>"
                match = re.search(
                    r'distance\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))', line
                )
                if match:
                    dist = float(match.group(1))
                    self.pub_dist.publish(Float32(data=dist))
                    self.get_logger().info(f"Distance = {dist:.2f} m")
        except Exception as e:
            self.get_logger().warn(f"Serial read failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Anchor1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
