#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial, re

class Anchor4Node(Node):
    def __init__(self):
        super().__init__('anchor4')

        self.declare_parameter('port', '/dev/Anchor4')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Anchor4 connected on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            raise

        self.pub_raw = self.create_publisher(String, '/uwb/anchor4/raw', 10)
        self.pub_dist = self.create_publisher(Float32, '/uwb/anchor4/distance_m', 10)
        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                self.pub_raw.publish(String(data=line))
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
    node = Anchor4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
