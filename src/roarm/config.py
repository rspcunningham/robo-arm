"""Joint calibration and hardware constants for RoArm-M2-S."""

from __future__ import annotations

import math
from dataclasses import dataclass

JOINT_NAMES = ("base", "shoulder", "elbow", "hand")

SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.05  # seconds


@dataclass(frozen=True)
class JointConfig:
    """Per-joint coordinate transform between user radians and firmware radians."""
    name: str
    sign: float       # +1 or -1
    offset: float     # added after sign flip (radians)


# Derived from the existing ROS driver:
#   firmware_to_user:  base = -fw,  shoulder = -fw,  elbow = +fw,  hand = pi - fw
#   user_to_firmware:  base = -usr, shoulder = -usr, elbow = +usr, hand = pi - usr
JOINTS = (
    JointConfig("base",     sign=-1.0, offset=0.0),
    JointConfig("shoulder", sign=-1.0, offset=0.0),
    JointConfig("elbow",    sign=+1.0, offset=0.0),
    JointConfig("hand",     sign=-1.0, offset=math.pi),
)


def firmware_to_user(firmware_values: tuple[float, ...]) -> tuple[float, ...]:
    """Convert firmware radians (b, s, e, t) to user radians."""
    return tuple(j.sign * fw + j.offset for j, fw in zip(JOINTS, firmware_values))


def user_to_firmware(user_values: tuple[float, ...]) -> tuple[float, ...]:
    """Convert user radians to firmware radians (b, s, e, t)."""
    return tuple(j.sign * (usr - j.offset) for j, usr in zip(JOINTS, user_values))
