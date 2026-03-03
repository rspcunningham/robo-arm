"""ESP32-CAM ROS2 driver node.

Fetches JPEG images over SPI from the ESP32-CAM and publishes them
as sensor_msgs/CompressedImage.
"""

import struct
import threading
import time
import zlib

import RPi.GPIO as GPIO
import rclpy
import spidev
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from nodes._util import run_node

CMD_GET_HEADER = 0x01
CMD_GET_FRAME_CHUNK = 0x02
CMD_GET_STATUS = 0x03

PROTOCOL_MAGIC = 0x314D4143  # "CAM1"
PROTOCOL_VERSION = 1
HEADER_STRUCT = struct.Struct("<IHHIII")
REQUEST_STRUCT = struct.Struct("<BBHI")
MAX_FRAME_LEN = 500_000
CHUNK_LEN = 4096
READY_PIN = 25
RESET_PIN = 27
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 4_000_000


class CamNode(Node):
    def __init__(self):
        super().__init__("cam")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            CompressedImage, "/camera/image/compressed", sensor_qos,
        )

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(READY_PIN, GPIO.IN)
        GPIO.setup(RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)

        self.spi = spidev.SpiDev()
        self._open_spi()

        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        self.get_logger().info("Cam node ready")

    def _open_spi(self):
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED_HZ
        self.spi.mode = 0

    def _close_spi(self):
        try:
            self.spi.close()
        except Exception:
            pass

    def _request(self, cmd: int, length: int = 0, offset: int = 0):
        payload = REQUEST_STRUCT.pack(cmd, 0, length, offset)
        self.spi.xfer2(list(payload))

    def _read_exact(self, nbytes: int) -> bytes:
        if nbytes <= 0:
            return b""
        return bytes(self.spi.xfer2([0] * nbytes))

    def _wait_ready(self, timeout: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout
        while self._running and time.monotonic() < deadline:
            if GPIO.input(READY_PIN):
                return True
            time.sleep(0.002)
        return False

    def _read_header(self) -> tuple[int, int, int, int, int] | None:
        self._request(CMD_GET_HEADER)
        raw = self._read_exact(HEADER_STRUCT.size)
        if len(raw) != HEADER_STRUCT.size:
            return None

        magic, version, flags, seq, length, crc32 = HEADER_STRUCT.unpack(raw)
        if magic != PROTOCOL_MAGIC or version != PROTOCOL_VERSION:
            return None
        if length <= 0 or length > MAX_FRAME_LEN:
            return None
        if not (flags & 0x0001):
            return None
        return seq, flags, length, crc32, magic

    def _read_frame(self, length: int) -> bytes | None:
        chunks = []
        offset = 0
        while offset < length and self._running:
            chunk_len = min(CHUNK_LEN, length - offset)
            self._request(CMD_GET_FRAME_CHUNK, chunk_len, offset)
            chunk = self._read_exact(chunk_len)
            if len(chunk) != chunk_len:
                return None
            chunks.append(chunk)
            offset += chunk_len
        return b"".join(chunks)

    def _fetch_frame(self) -> bytes | None:
        if not self._wait_ready():
            return None

        header = self._read_header()
        if header is None:
            return None

        _seq, _flags, length, crc32, _magic = header
        frame = self._read_frame(length)
        if frame is None:
            return None
        if zlib.crc32(frame) & 0xFFFFFFFF != crc32:
            return None
        return frame

    def _reset_esp32(self):
        GPIO.output(RESET_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(RESET_PIN, GPIO.HIGH)
        time.sleep(1.0)

    def _recover_link(self):
        self.get_logger().warning("Recovering SPI link")
        self._close_spi()
        time.sleep(0.1)
        try:
            self._open_spi()
            return
        except OSError:
            pass

        self._reset_esp32()
        self._close_spi()
        time.sleep(0.1)
        self._open_spi()

    def _reader_loop(self):
        failures = 0
        while self._running:
            frame = self._fetch_frame()
            if frame is None:
                failures += 1
                if failures >= 5:
                    try:
                        self._recover_link()
                    except OSError:
                        time.sleep(0.5)
                    failures = 0
                continue

            failures = 0
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = frame
            self.pub.publish(msg)

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=3)
        self._close_spi()
        GPIO.cleanup((READY_PIN, RESET_PIN))
        super().destroy_node()


def main():
    rclpy.init()
    run_node(CamNode())
