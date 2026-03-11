"""USB webcam ROS2 node (V4L2/OpenCV).

Publishes uncompressed image frames and basic camera metadata:
- /cam0/image_raw (sensor_msgs/Image)
- /cam0/camera_info (sensor_msgs/CameraInfo)
"""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from nodes._util import publish_or_ignore_shutdown, run_node

IMAGE_TOPIC = "/cam0/image_raw"
CAMERA_INFO_TOPIC = "/cam0/camera_info"
COMPRESSED_IMAGE_TOPIC = "/cam0/image/compressed"

DEFAULT_DEVICE = "/dev/v4l/by-id/usb-MACROSILICON_Guermok_USB2_Video_20250820-video-index0"
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30.0
DEFAULT_PIXEL_FORMAT = "MJPG"
DEFAULT_FRAME_ID = "cam0_optical"
DEFAULT_JPEG_QUALITY = 80


class Cam0Node(Node):
    def __init__(
        self,
        device: str,
        width: int,
        height: int,
        fps: float,
        pixel_format: str,
        jpeg_quality: int,
        frame_id: str,
    ):
        super().__init__("cam0")

        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "OpenCV is required for USB cam0. Install with: cd pi && uv sync"
            ) from exc

        self._cv2 = cv2
        self._frame_id = frame_id
        self._jpeg_quality = int(max(1, min(100, jpeg_quality)))
        self._running = True
        self._warning_interval_sec = 2.0
        self._last_warning_monotonic = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_pub = self.create_publisher(Image, IMAGE_TOPIC, sensor_qos)
        self._camera_info_pub = self.create_publisher(CameraInfo, CAMERA_INFO_TOPIC, sensor_qos)
        self._compressed_pub = self.create_publisher(CompressedImage, COMPRESSED_IMAGE_TOPIC, sensor_qos)

        self._cap = self._open_capture(
            device=device,
            width=width,
            height=height,
            fps=fps,
            pixel_format=pixel_format,
        )

        # Publish on a timer instead of a tight loop so ROS shutdown remains responsive.
        self._timer = self.create_timer(1.0 / max(fps, 1.0), self._capture_once)

        actual_width = int(self._cap.get(self._cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._cap.get(self._cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self._cap.get(self._cv2.CAP_PROP_FPS))
        self.get_logger().info(
            "Cam0 ready "
            f"(device={device}, {actual_width}x{actual_height}, format={pixel_format}, "
            f"fps~{actual_fps:g}, jpeg_quality={self._jpeg_quality})"
        )

    def _warn_throttled(self, message: str):
        now = time.monotonic()
        if now - self._last_warning_monotonic >= self._warning_interval_sec:
            self._last_warning_monotonic = now
            self.get_logger().warning(message)

    def _open_capture(self, device: str, width: int, height: int, fps: float, pixel_format: str):
        cap = self._cv2.VideoCapture(device, self._cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(
                f"Failed to open USB camera device {device}. "
                "Verify permissions and that the device exists."
            )

        if pixel_format:
            fourcc = self._cv2.VideoWriter_fourcc(*pixel_format)
            cap.set(self._cv2.CAP_PROP_FOURCC, fourcc)

        if width > 0:
            cap.set(self._cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height > 0:
            cap.set(self._cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if fps > 0:
            cap.set(self._cv2.CAP_PROP_FPS, float(fps))

        # Keep capture latency low.
        cap.set(self._cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _capture_once(self):
        if not self._running or not rclpy.ok():
            return

        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._warn_throttled("Failed to read frame from USB camera")
            return

        if frame.ndim != 3 or frame.shape[2] != 3:
            self._warn_throttled(f"Unexpected frame shape from USB camera: {frame.shape}")
            return

        if not frame.flags.c_contiguous:
            frame = frame.copy(order="C")

        height, width = frame.shape[:2]
        step = width * 3

        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id
        image.height = int(height)
        image.width = int(width)
        image.encoding = "bgr8"
        image.is_bigendian = 0
        image.step = int(step)
        image.data = frame.tobytes()

        if not publish_or_ignore_shutdown(self._image_pub, image):
            return

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.height = int(height)
        info.width = int(width)
        info.distortion_model = "plumb_bob"
        info.d = []
        info.k = [0.0] * 9
        info.r = [0.0] * 9
        info.p = [0.0] * 12
        publish_or_ignore_shutdown(self._camera_info_pub, info)

        ok, encoded = self._cv2.imencode(
            ".jpg",
            frame,
            [self._cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
        )
        if not ok:
            self._warn_throttled("Failed to encode JPEG from USB camera frame")
            return

        compressed = CompressedImage()
        compressed.header.stamp = stamp
        compressed.header.frame_id = self._frame_id
        compressed.format = "jpeg"
        compressed.data = encoded.tobytes()
        publish_or_ignore_shutdown(self._compressed_pub, compressed)

    def destroy_node(self):
        self._running = False
        try:
            self._cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="USB camera ROS2 node (cam0 via V4L2/OpenCV)")
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help=f"V4L2 device path (default: {DEFAULT_DEVICE})",
    )
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Requested width")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Requested height")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="Requested FPS")
    parser.add_argument(
        "--pixel-format",
        default=DEFAULT_PIXEL_FORMAT,
        help="V4L2 fourcc pixel format (default: MJPG)",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=DEFAULT_JPEG_QUALITY,
        help=f"JPEG quality for /cam0/image/compressed in [1,100] (default: {DEFAULT_JPEG_QUALITY})",
    )
    parser.add_argument(
        "--frame-id",
        default=DEFAULT_FRAME_ID,
        help=f"ROS frame id for Image/CameraInfo headers (default: {DEFAULT_FRAME_ID})",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Cam0Node(
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        pixel_format=args.pixel_format,
        jpeg_quality=args.jpeg_quality,
        frame_id=args.frame_id,
    )
    run_node(node)


if __name__ == "__main__":
    main()
