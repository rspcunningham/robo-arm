from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from serial.tools import list_ports

LEFT_SERIAL = "acff0192339bef11b2e7b69061ce3355"
RIGHT_SERIAL = "24ee9dd6f400f0119b83c3295c2a50c9"


@dataclass(frozen=True)
class ArmPorts:
    left: str
    right: str


def _find_port_by_serial(target_serial: str, ports: Iterable[object]) -> str | None:
    for port in ports:
        if getattr(port, "serial_number", None) == target_serial:
            return str(getattr(port, "device"))
    return None


def find_arm_ports() -> ArmPorts:
    ports = list(list_ports.comports())
    left = _find_port_by_serial(LEFT_SERIAL, ports)
    right = _find_port_by_serial(RIGHT_SERIAL, ports)

    missing: list[str] = []
    if left is None:
        missing.append(f"left ({LEFT_SERIAL})")
    if right is None:
        missing.append(f"right ({RIGHT_SERIAL})")
    if missing:
        raise RuntimeError(f"Could not find arm port(s): {', '.join(missing)}")

    return ArmPorts(left=left, right=right)
