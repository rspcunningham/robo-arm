"""Save a single frame from the camera topic as a JPEG file."""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class SnapNode(Node):
    def __init__(self, output: str):
        super().__init__("cam_snap")
        self.output = output
        self.done = False
        self.create_subscription(
            CompressedImage, "/camera/image/compressed", self._on_image, 10,
        )

    def _on_image(self, msg: CompressedImage):
        if self.done:
            return
        self.done = True
        with open(self.output, "wb") as f:
            f.write(bytes(msg.data))
        self.get_logger().info(f"Saved {self.output} ({len(msg.data)} bytes)")


def main():
    output = sys.argv[1] if len(sys.argv) > 1 else "frame.jpg"

    rclpy.init()
    node = SnapNode(output)
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
