"""CSI camera ROS2 node using libcamera Python bindings.

This node publishes uncompressed image frames and basic camera metadata:
- /cam0/image_raw (sensor_msgs/Image)
- /cam0/camera_info (sensor_msgs/CameraInfo)
"""

from __future__ import annotations

import argparse
import glob
import importlib.util
import mmap
import os
import selectors
import site
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from nodes._util import publish_or_ignore_shutdown, run_node

IMAGE_TOPIC = "/cam0/image_raw"
CAMERA_INFO_TOPIC = "/cam0/camera_info"

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 360
DEFAULT_FPS = 20.0
DEFAULT_PIXEL_FORMAT = "RGB888"
DEFAULT_FRAME_ID = "cam0_optical"
APPLY_ROTATION_180 = True
FORCE_MAX_FOV_CROP = True
AUTOFOCUS_ON_STARTUP = True
AUTOFOCUS_WINDOW_CENTER_X = 0.50
AUTOFOCUS_WINDOW_CENTER_Y = 0.72
AUTOFOCUS_WINDOW_WIDTH_FRAC = 0.35
AUTOFOCUS_WINDOW_HEIGHT_FRAC = 0.35


def _import_libcamera():
    if importlib.util.find_spec("libcamera") is None:
        # Ubuntu's python3-libcamera may be installed in an arch-specific
        # system site-packages path not present in uv virtualenv sys.path.
        for path in sorted(glob.glob("/usr/lib/*-linux-gnu/python*/site-packages")):
            if os.path.isdir(os.path.join(path, "libcamera")):
                site.addsitedir(path)

    try:
        import libcamera as libcam
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "python3-libcamera is required for cam0. Install: sudo apt install -y python3-libcamera"
        ) from exc
    return libcam


def _encoding_from_pixel_format(pixel_format_name: str) -> str:
    mapping = {
        "RGB888": "bgr8",  # libcamera naming is little-endian oriented.
        "BGR888": "rgb8",
        "XRGB8888": "bgra8",
        "XBGR8888": "rgba8",
        "Y8": "mono8",
    }
    encoding = mapping.get(pixel_format_name)
    if encoding is None:
        raise RuntimeError(
            f"Unsupported pixel format for ROS publishing: {pixel_format_name}. "
            "Use RGB888, BGR888, XRGB8888, XBGR8888, or Y8."
        )
    return encoding


def _bytes_per_pixel(encoding: str) -> int:
    if encoding in {"rgb8", "bgr8"}:
        return 3
    if encoding in {"rgba8", "bgra8"}:
        return 4
    if encoding == "mono8":
        return 1
    raise RuntimeError(f"Unsupported ROS encoding: {encoding}")


class Cam0Node(Node):
    def __init__(
        self,
        camera_index: int,
        width: int,
        height: int,
        fps: float,
        pixel_format: str,
        frame_id: str,
    ):
        super().__init__("cam0")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_pub = self.create_publisher(Image, IMAGE_TOPIC, sensor_qos)
        self._camera_info_pub = self.create_publisher(CameraInfo, CAMERA_INFO_TOPIC, sensor_qos)

        self._frame_id = frame_id
        self._fps = fps
        self._running = True
        self._last_warning_monotonic = 0.0
        self._warning_interval_sec = 2.0

        self._libcam = _import_libcamera()
        self._cm = self._libcam.CameraManager.singleton()

        self._camera = None
        self._stream = None
        self._allocator = None
        self._requests = []
        self._mapped_buffers = {}
        self._selector = selectors.DefaultSelector()

        self._width = width
        self._height = height
        self._step = 0
        self._encoding = ""
        self._pixel_format_name = ""
        self._row_width = 0
        self._frame_duration_us = 0
        self._fov_crop = None
        self._af_window = None
        self._af_trigger_sent = False

        self._capture_thread = None

        try:
            self._setup_camera(camera_index, width, height, fps, pixel_format)
            self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._capture_thread.start()
        except Exception:
            self._shutdown_camera()
            raise

        self.get_logger().info(
            "Cam0 ready "
            f"(camera={camera_index}, {self._width}x{self._height}, "
            f"format={self._pixel_format_name}, encoding={self._encoding}, fps~{fps:g})"
        )

    def _setup_camera(self, camera_index: int, width: int, height: int, fps: float, pixel_format: str):
        cameras = list(self._cm.cameras)
        if not cameras:
            raise RuntimeError("No libcamera cameras were detected")
        if camera_index < 0 or camera_index >= len(cameras):
            raise RuntimeError(
                f"Camera index {camera_index} is out of range (detected {len(cameras)} camera(s))"
            )

        self._camera = cameras[camera_index]
        self._camera.acquire()

        config = self._camera.generate_configuration([self._libcam.StreamRole.Viewfinder])
        if config is None:
            raise RuntimeError("Failed to generate libcamera configuration")

        stream_cfg = config.at(0)
        stream_cfg.size.width = int(width)
        stream_cfg.size.height = int(height)
        stream_cfg.pixel_format = self._libcam.PixelFormat(pixel_format)
        if APPLY_ROTATION_180 and hasattr(config, "orientation"):
            config.orientation = self._libcam.Orientation.Rotate180

        status = config.validate()
        if status == self._libcam.CameraConfiguration.Status.Invalid:
            raise RuntimeError("Requested camera configuration is invalid")
        if status == self._libcam.CameraConfiguration.Status.Adjusted:
            self.get_logger().warning("Camera configuration was adjusted by libcamera")

        self._camera.configure(config)

        stream_cfg = config.at(0)
        self._stream = stream_cfg.stream

        self._width = int(stream_cfg.size.width)
        self._height = int(stream_cfg.size.height)
        self._pixel_format_name = str(stream_cfg.pixel_format)
        self._encoding = _encoding_from_pixel_format(self._pixel_format_name)

        fallback_step = self._width * _bytes_per_pixel(self._encoding)
        stride = getattr(stream_cfg, "stride", None)
        self._step = int(stride) if stride else fallback_step
        self._row_width = self._width * _bytes_per_pixel(self._encoding)

        self._allocator = self._libcam.FrameBufferAllocator(self._camera)
        allocated = self._allocator.allocate(self._stream)
        if allocated <= 0:
            raise RuntimeError("Failed to allocate libcamera frame buffers")

        buffers = list(self._allocator.buffers(self._stream))
        if not buffers:
            raise RuntimeError("libcamera did not provide any frame buffers")

        frame_duration_us = 0
        if fps > 0:
            frame_duration_us = max(1, int(1_000_000.0 / fps))
        self._frame_duration_us = frame_duration_us
        self._fov_crop = self._select_full_fov_crop()
        self._af_window = self._build_autofocus_window()

        self._requests = []
        self._mapped_buffers = {}
        for i, buffer in enumerate(buffers):
            request = self._camera.create_request(i)
            if request is None:
                raise RuntimeError("Failed to create libcamera request")
            request.add_buffer(self._stream, buffer)

            self._requests.append(request)
            plane = buffer.planes[0]
            self._mapped_buffers[id(buffer)] = mmap.mmap(
                plane.fd,
                plane.length,
                flags=mmap.MAP_SHARED,
                prot=mmap.PROT_READ,
                offset=plane.offset,
            )

        self._selector.register(self._cm.event_fd, selectors.EVENT_READ)

        self._camera.start()
        for request in self._requests:
            self._configure_request_controls(request)
            self._camera.queue_request(request)

    def _camera_property(self, name: str):
        if self._camera is None:
            return None
        for prop, value in self._camera.properties.items():
            if getattr(prop, "name", str(prop)) == name:
                return value
        return None

    def _select_full_fov_crop(self):
        if not FORCE_MAX_FOV_CROP:
            return None
        active_areas = self._camera_property("PixelArrayActiveAreas")
        if isinstance(active_areas, tuple) and active_areas:
            return active_areas[0]
        return active_areas

    def _build_autofocus_window(self):
        if not AUTOFOCUS_ON_STARTUP:
            return None
        window_w = max(1, int(self._width * AUTOFOCUS_WINDOW_WIDTH_FRAC))
        window_h = max(1, int(self._height * AUTOFOCUS_WINDOW_HEIGHT_FRAC))
        center_x = int(self._width * AUTOFOCUS_WINDOW_CENTER_X)
        center_y = int(self._height * AUTOFOCUS_WINDOW_CENTER_Y)
        x = max(0, min(self._width - window_w, center_x - window_w // 2))
        y = max(0, min(self._height - window_h, center_y - window_h // 2))
        return self._libcam.Rectangle(x, y, window_w, window_h)

    def _set_control(self, request, name: str, value):
        if not hasattr(self._libcam.controls, name):
            return
        request.set_control(getattr(self._libcam.controls, name), value)

    def _configure_request_controls(self, request):
        try:
            if self._frame_duration_us > 0:
                self._set_control(
                    request,
                    "FrameDurationLimits",
                    (self._frame_duration_us, self._frame_duration_us),
                )

            if self._fov_crop is not None:
                self._set_control(request, "ScalerCrop", self._fov_crop)

            if AUTOFOCUS_ON_STARTUP:
                self._set_control(request, "AfMode", self._libcam.controls.AfModeEnum.Auto)
                self._set_control(request, "AfRange", self._libcam.controls.AfRangeEnum.Macro)
                self._set_control(request, "AfSpeed", self._libcam.controls.AfSpeedEnum.Normal)
                if self._af_window is not None:
                    self._set_control(request, "AfMetering", self._libcam.controls.AfMeteringEnum.Windows)
                    self._set_control(request, "AfWindows", [self._af_window])
                if not self._af_trigger_sent:
                    self._set_control(request, "AfTrigger", self._libcam.controls.AfTriggerEnum.Start)
                    self._af_trigger_sent = True
                    self.get_logger().info(
                        f"Triggered autofocus on startup (window={self._af_window})"
                    )
        except Exception as exc:
            self._warn_throttled(f"Failed to set camera controls: {exc}")

    def _warn_throttled(self, message: str):
        now = time.monotonic()
        if now - self._last_warning_monotonic >= self._warning_interval_sec:
            self._last_warning_monotonic = now
            self.get_logger().warning(message)

    def _capture_loop(self):
        while self._running and rclpy.ok():
            events = self._selector.select(timeout=0.5)
            if not events:
                continue

            requests = self._cm.get_ready_requests()
            for request in requests:
                try:
                    self._process_request(request)
                except Exception as exc:
                    if self._running:
                        self._warn_throttled(f"Capture processing error: {exc}")

                if not self._running:
                    break

                try:
                    request.reuse()
                    self._camera.queue_request(request)
                except Exception as exc:
                    if self._running:
                        self.get_logger().error(f"Failed to re-queue camera request: {exc}")
                    self._running = False
                    break

    def _process_request(self, request):
        if request.status != self._libcam.Request.Status.Complete:
            return

        try:
            buffer = request.buffers[self._stream]
        except Exception:
            return

        mapped = self._mapped_buffers.get(id(buffer))
        if mapped is None:
            return

        if not buffer.metadata.planes:
            return

        bytes_used = int(buffer.metadata.planes[0].bytes_used)
        if bytes_used <= 0:
            return

        expected_bytes = self._step * self._height
        if bytes_used < expected_bytes:
            self._warn_throttled(
                f"Dropping short frame ({bytes_used} < expected {expected_bytes} bytes)"
            )
            return

        frame = bytes(memoryview(mapped)[:expected_bytes])
        self._publish_frame(frame)

    def _publish_frame(self, frame: bytes):
        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id
        image.height = self._height
        image.width = self._width
        image.encoding = self._encoding
        image.is_bigendian = 0
        image.step = self._step
        image.data = frame

        if not publish_or_ignore_shutdown(self._image_pub, image):
            return

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.height = self._height
        info.width = self._width
        info.distortion_model = "plumb_bob"
        info.d = []
        info.k = [0.0] * 9
        info.r = [0.0] * 9
        info.p = [0.0] * 12
        publish_or_ignore_shutdown(self._camera_info_pub, info)

    def _shutdown_camera(self):
        try:
            self._selector.close()
        except Exception:
            pass

        if self._camera is not None:
            try:
                self._camera.stop()
            except Exception:
                pass
            try:
                self._camera.release()
            except Exception:
                pass

        self._camera = None
        for mapped in self._mapped_buffers.values():
            try:
                mapped.close()
            except Exception:
                pass
        self._mapped_buffers.clear()

    def destroy_node(self):
        self._running = False

        if self._capture_thread is not None:
            self._capture_thread.join(timeout=2.0)

        self._shutdown_camera()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="CSI camera ROS2 node (python3-libcamera)")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Requested width")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Requested height")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="Target FPS")
    parser.add_argument(
        "--pixel-format",
        default=DEFAULT_PIXEL_FORMAT,
        help="Requested libcamera pixel format (default: RGB888)",
    )
    parser.add_argument(
        "--frame-id",
        default=DEFAULT_FRAME_ID,
        help=f"ROS frame id for Image/CameraInfo headers (default: {DEFAULT_FRAME_ID})",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Cam0Node(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        pixel_format=args.pixel_format,
        frame_id=args.frame_id,
    )
    run_node(node)


if __name__ == "__main__":
    main()
