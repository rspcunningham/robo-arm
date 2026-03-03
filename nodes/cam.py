"""ESP32-CAM ROS2 driver node.

Reads framed JPEG images over UART and publishes them
as sensor_msgs/CompressedImage.
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import serial

from nodes._util import run_node

MAGIC = b"\xaa\x55\x01\x00"


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


class CamNode(Node):
    def __init__(self):
        super().__init__("cam")

        self.ser = serial.Serial("/dev/ttyAMA0", 2000000, timeout=2)
        self.pub = self.create_publisher(
            CompressedImage, "/camera/image/compressed", 10,
        )

        # Configure and start streaming
        self.ser.reset_input_buffer()
        self.ser.write(b"RESOLUTION VGA\n")
        self.ser.write(b"QUALITY 20\n")
        self.ser.write(b"START\n")

        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        self.get_logger().info("Cam node ready")

    # -- frame reading (reused from receiver.py) -----------------------

    def _read_frame(self) -> bytes | None:
        """Wait for and receive a single framed JPEG."""
        buf = b""
        deadline = time.monotonic() + 5.0

        # Scan for magic bytes
        while self._running and time.monotonic() < deadline:
            byte = self.ser.read(1)
            if not byte:
                continue
            buf += byte
            if len(buf) > 256 and MAGIC not in buf[-256:]:
                buf = buf[-4:]
            if buf.endswith(MAGIC):
                break
        else:
            return None

        # Read length (4 bytes, little-endian)
        raw_len = self.ser.read(4)
        if len(raw_len) < 4:
            return None
        frame_len = struct.unpack("<I", raw_len)[0]
        if frame_len > 500_000:
            return None

        # Read JPEG payload
        jpeg_data = b""
        remaining = frame_len
        while remaining > 0 and time.monotonic() < deadline:
            chunk = self.ser.read(min(remaining, 4096))
            if not chunk:
                break
            jpeg_data += chunk
            remaining -= len(chunk)

        if len(jpeg_data) != frame_len:
            return None

        # Verify CRC
        raw_crc = self.ser.read(2)
        if len(raw_crc) < 2:
            return None
        if struct.unpack("<H", raw_crc)[0] != crc16(jpeg_data):
            return None

        return jpeg_data

    # -- reader thread -------------------------------------------------

    def _reader_loop(self):
        while self._running:
            frame = self._read_frame()
            if frame is None:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = frame
            self.pub.publish(msg)

    # -- lifecycle -----------------------------------------------------

    def destroy_node(self):
        self._running = False
        self.ser.write(b"STOP\n")
        self._thread.join(timeout=3)
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    run_node(CamNode())
