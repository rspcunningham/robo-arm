from __future__ import annotations

from roarm.arm import Arm
from roarm.ports import find_arm_ports


def main() -> None:
    ports = find_arm_ports()
    print(f"left:  {ports.left}")
    print(f"right: {ports.right}")

    with Arm(ports.left) as left_arm, Arm(ports.right) as right_arm:
        left_state = left_arm.read_state()
        right_state = right_arm.read_state()
        print(f"left_state:  {left_state}")
        print(f"right_state: {right_state}")


if __name__ == "__main__":
    main()
