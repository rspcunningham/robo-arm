"""ESP32-CAM ROS2 driver node.

Continuously drains JPEG frames over SPI from the ESP32-CAM and publishes them
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

CMD_NOP = 0x00
CMD_GET_HEADER = 0x01
CMD_GET_FRAME_CHUNK = 0x02
CMD_GET_STATUS = 0x03
CMD_ACK_FRAME = 0x04

PROTOCOL_MAGIC = 0x314D4143  # "CAM1"
PROTOCOL_VERSION = 1
RESPONSE_MAGIC = 0xA55AA55A

HEADER_STRUCT = struct.Struct("<IHHIII")
REQUEST_STRUCT = struct.Struct("<BBHI")
RESPONSE_STRUCT = struct.Struct("<IHBB")

MAX_FRAME_LEN = 500_000
CHUNK_LEN = 4088
TRANSFER_LEN = RESPONSE_STRUCT.size + CHUNK_LEN

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

        self._running = True
        self._last_seq = 0

        self.spi = spidev.SpiDev()
        self._open_spi()

        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        self.get_logger().info("Cam node ready")

    def _open_spi(self):
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED_HZ
        self.spi.mode = 0
        self.spi.bits_per_word = 8
        self._prime_pipeline()

    def _close_spi(self):
        try:
            self.spi.close()
        except Exception:
            pass

    def _prime_pipeline(self):
        self._transfer(CMD_NOP)

    def _transfer(self, cmd: int, length: int = 0, offset: int = 0):
        if not self._wait_ready(timeout=0.5):
            return None
        tx = bytearray(TRANSFER_LEN)
        REQUEST_STRUCT.pack_into(tx, 0, cmd, 0, length, offset)
        rx = bytes(self.spi.xfer2(list(tx)))

        magic, resp_len, resp_cmd, flags = RESPONSE_STRUCT.unpack_from(rx, 0)
        if magic != RESPONSE_MAGIC or resp_len > CHUNK_LEN:
            return None

        payload = rx[RESPONSE_STRUCT.size:RESPONSE_STRUCT.size + resp_len]
        return resp_cmd, flags, payload

    def _wait_ready(self, timeout: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout
        while self._running and time.monotonic() < deadline:
            if GPIO.input(READY_PIN):
                return True
            time.sleep(0.0005)
        return False

    def _read_header(self):
        self._transfer(CMD_GET_HEADER)
        resp = self._transfer(CMD_NOP)
        if resp is None:
            return None

        resp_cmd, _flags, payload = resp
        if resp_cmd != CMD_GET_HEADER or len(payload) != HEADER_STRUCT.size:
            return None

        magic, version, header_flags, seq, length, crc32 = HEADER_STRUCT.unpack(payload)
        if magic != PROTOCOL_MAGIC or version != PROTOCOL_VERSION:
            return None
        if not (header_flags & 0x0001):
            return None
        if length <= 0 or length > MAX_FRAME_LEN:
            return None

        return seq, length, crc32

    def _read_frame(self, seq: int, length: int) -> bytes | None:
        chunks = []
        offset = 0
        pending_len = min(CHUNK_LEN, length)

        resp = self._transfer(CMD_GET_FRAME_CHUNK, pending_len, offset)
        if resp is None:
            return None

        offset += pending_len

        while offset < length and self._running:
            next_len = min(CHUNK_LEN, length - offset)
            resp = self._transfer(CMD_GET_FRAME_CHUNK, next_len, offset)
            if resp is None:
                return None

            resp_cmd, _flags, payload = resp
            if resp_cmd != CMD_GET_FRAME_CHUNK or len(payload) != pending_len:
                return None
            chunks.append(payload)

            pending_len = next_len
            offset += next_len

        resp = self._transfer(CMD_ACK_FRAME, 0, seq)
        if resp is None:
            return None

        resp_cmd, _flags, payload = resp
        if resp_cmd != CMD_GET_FRAME_CHUNK or len(payload) != pending_len:
            return None
        chunks.append(payload)

        return b"".join(chunks)

    def _fetch_frame(self) -> bytes | None:
        header = self._read_header()
        if header is None:
            self._prime_pipeline()
            return None

        seq, length, crc32 = header
        if seq == self._last_seq:
            self._prime_pipeline()
            return None

        frame = self._read_frame(seq, length)
        if frame is None:
            self._prime_pipeline()
            return None
        if zlib.crc32(frame) & 0xFFFFFFFF != crc32:
            self._prime_pipeline()
            return None

        self._last_seq = seq
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
            if not self._running or not rclpy.ok():
                break
            try:
                self.pub.publish(msg)
            except Exception:
                if not self._running or not rclpy.ok():
                    break
                raise

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=3)
        self._close_spi()
        GPIO.cleanup((READY_PIN, RESET_PIN))
        super().destroy_node()


def main():
    rclpy.init()
    run_node(CamNode())
