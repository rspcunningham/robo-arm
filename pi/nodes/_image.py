"""Helpers for converting ROS Image messages into RGB/JPEG payloads."""

from __future__ import annotations

import io

import numpy as np
from PIL import Image
from sensor_msgs.msg import Image as RosImage


def ros_image_to_rgb_array(msg: RosImage) -> np.ndarray:
    if msg.width <= 0 or msg.height <= 0:
        raise ValueError("image has invalid dimensions")
    if msg.step <= 0:
        raise ValueError("image has invalid row stride")

    total_bytes = msg.step * msg.height
    if len(msg.data) < total_bytes:
        raise ValueError("image payload is smaller than step*height")

    rows = np.frombuffer(msg.data, dtype=np.uint8)[:total_bytes].reshape(msg.height, msg.step)
    encoding = msg.encoding.lower()

    if encoding in {"rgb8", "bgr8"}:
        row_width = msg.width * 3
        if msg.step < row_width:
            raise ValueError("row stride is too small for 3-channel image")
        image = rows[:, :row_width].reshape(msg.height, msg.width, 3).copy()
        if encoding == "bgr8":
            image = image[:, :, ::-1]
        return image

    if encoding in {"rgba8", "bgra8"}:
        row_width = msg.width * 4
        if msg.step < row_width:
            raise ValueError("row stride is too small for 4-channel image")
        image = rows[:, :row_width].reshape(msg.height, msg.width, 4).copy()
        if encoding == "bgra8":
            image = image[:, :, [2, 1, 0, 3]]
        return image[:, :, :3]

    if encoding == "mono8":
        row_width = msg.width
        if msg.step < row_width:
            raise ValueError("row stride is too small for mono image")
        mono = rows[:, :row_width].reshape(msg.height, msg.width)
        return np.repeat(mono[:, :, None], 3, axis=2)

    raise ValueError(f"unsupported image encoding: {msg.encoding!r}")


def ros_image_to_rgb_pil(msg: RosImage) -> Image.Image:
    return Image.fromarray(ros_image_to_rgb_array(msg), mode="RGB")


def ros_image_to_jpeg_bytes(msg: RosImage, quality: int = 85) -> bytes:
    image = ros_image_to_rgb_pil(msg)
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG", quality=quality)
    return buffer.getvalue()
