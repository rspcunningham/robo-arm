from __future__ import annotations

from dataclasses import dataclass

from roarm.arm import Arm, ArmState
from roarm.ports import find_arm_ports


@dataclass
class DualArmState:
    left: ArmState | None
    right: ArmState | None


class DualArm:
    def __init__(self, left_port: str, right_port: str):
        self.left = Arm(left_port)
        self.right = Arm(right_port)

    @classmethod
    def from_known_ports(cls) -> "DualArm":
        ports = find_arm_ports()
        return cls(left_port=ports.left, right_port=ports.right)

    def connect(self) -> None:
        self.left.connect()
        self.right.connect()

    def disconnect(self) -> None:
        self.left.disconnect()
        self.right.disconnect()

    def __enter__(self) -> "DualArm":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.disconnect()

    def read_state(self) -> DualArmState:
        return DualArmState(
            left=self.left.read_state(),
            right=self.right.read_state(),
        )

    def set_torque(self, enabled: bool) -> None:
        self.left.set_torque(enabled)
        self.right.set_torque(enabled)

    def lock(self) -> None:
        self.set_torque(True)

    def unlock(self) -> None:
        self.set_torque(False)
