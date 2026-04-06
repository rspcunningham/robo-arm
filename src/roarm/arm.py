"""Direct serial interface to RoArm-M2-S via ESP32 JSON protocol."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

import serial

from roarm.config import (
    JOINT_NAMES,
    SERIAL_BAUDRATE,
    SERIAL_TIMEOUT,
    firmware_to_user,
    user_to_firmware,
)


@dataclass
class ArmState:
    position: tuple[float, ...]     # radians, user frame
    load: tuple[float, ...]         # raw torque values from firmware
    timestamp: float                # time.monotonic()


class Arm:
    def __init__(self, port: str, baudrate: int = SERIAL_BAUDRATE):
        self._port = port
        self._baudrate = baudrate
        self._ser: serial.Serial | None = None

    # -- lifecycle --

    def connect(self):
        self._ser = serial.Serial(self._port, self._baudrate, timeout=SERIAL_TIMEOUT)
        self._ser.rts = False
        self._ser.dtr = False
        # Silence firmware debug chatter.
        self._send({"T": 605, "cmd": 0})
        self._ser.reset_input_buffer()

    def disconnect(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()

    # -- commands --

    def read_state(self) -> ArmState | None:
        """Query servo feedback. Returns None if no valid response."""
        self._send({"T": 105})
        data = self._read_response()
        if data is None or data.get("T") != 1051:
            return None
        fw_pos = (data["b"], data["s"], data["e"], data["t"])
        return ArmState(
            position=firmware_to_user(fw_pos),
            load=(
                float(data.get("torB", 0)),
                float(data.get("torS", 0)),
                float(data.get("torE", 0)),
                float(data.get("torH", 0)),
            ),
            timestamp=time.monotonic(),
        )

    def move(
        self,
        position: tuple[float, ...],
        speed: float = 0.0,
        acceleration: float = 0.0,
    ):
        """Command all joints. Position in user radians."""
        fw = user_to_firmware(position)
        self._send({
            "T": 102,
            "base": fw[0],
            "shoulder": fw[1],
            "elbow": fw[2],
            "hand": fw[3],
            "spd": speed,
            "acc": acceleration,
        })

    def set_torque(self, enabled: bool):
        self._send({"T": 210, "cmd": 1 if enabled else 0})

    def set_light(self, enabled: bool):
        self._send({"T": 114, "led": 255 if enabled else 0})

    def emergency_stop(self):
        self._send({"T": 0})

    def home(self):
        """Move to zero pose."""
        self.move((0.0, 0.0, 0.0, 0.0))

    # -- internals --

    def _send(self, obj: dict):
        assert self._ser is not None, "Not connected"
        self._ser.write((json.dumps(obj) + "\n").encode())

    def _read_response(self) -> dict | None:
        assert self._ser is not None, "Not connected"
        line = self._ser.readline()
        if not line:
            return None
        try:
            return json.loads(line)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None
