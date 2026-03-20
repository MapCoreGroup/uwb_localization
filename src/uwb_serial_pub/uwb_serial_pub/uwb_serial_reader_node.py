#!/usr/bin/env python3

import re
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

import serial


class UwbSerialReaderNode(Node):
    """
    Generic UWB serial reader.

    Defaults are aligned with the symlinks created by `99-sensors.rules`:
      - anchor_id=1 -> /dev/Anchor1
      - anchor_id=2 -> /dev/Anchor2
      - anchor_id=3 -> /dev/Anchor3
      - anchor_id=4 -> /dev/Anchor4
    """

    # ESP32 output is expected to include: "distance = <number>"
    _DISTANCE_RE = re.compile(r"distance\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))")

    def __init__(self):
        super().__init__("uwb_serial_reader_node")

        self.declare_parameter("anchor_id", 1)
        # If empty, we derive the port from anchor_id.
        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 115200)

        anchor_id = int(self.get_parameter("anchor_id").value)
        port: str = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)

        if not (1 <= anchor_id <= 4):
            raise ValueError(f"anchor_id must be in [1,4], got {anchor_id}")

        if not port:
            port = f"/dev/Anchor{anchor_id}"

        self.anchor_id = anchor_id
        self.port = port
        self.baudrate = baudrate

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(
                f"Connected anchor{anchor_id} on {self.port} @ {self.baudrate}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise

        # Publish raw serial line and extracted distance.
        self.pub_raw = self.create_publisher(
            String, f"/uwb/anchor{anchor_id}/raw", 10
        )
        self.pub_dist = self.create_publisher(
            Float32, f"/uwb/anchor{anchor_id}/distance_m", 10
        )

        self.timer = self.create_timer(0.05, self.read_serial)  # ~20 Hz

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                return

            self.pub_raw.publish(String(data=line))

            match = self._DISTANCE_RE.search(line)
            if not match:
                return

            dist = float(match.group(1))
            self.pub_dist.publish(Float32(data=dist))
        except Exception as e:
            # Avoid spamming errors at high frequency; just warn.
            self.get_logger().warn(f"Serial read failed: {e}")

    def destroy_node(self):
        # Close serial port cleanly.
        try:
            if hasattr(self, "ser") and self.ser:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args: Optional[list[str]] = None):
    rclpy.init(args=args)
    node = UwbSerialReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

