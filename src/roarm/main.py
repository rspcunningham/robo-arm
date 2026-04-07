from __future__ import annotations

import argparse
import time

from roarm.dual import DualArm
from roarm.ports import find_arm_ports


def _cmd_status() -> None:
    ports = find_arm_ports()
    print(f"left:  {ports.left}")
    print(f"right: {ports.right}")

    with DualArm.from_known_ports() as arms:
        state = arms.read_state()
        print(f"left_state:  {state.left}")
        print(f"right_state: {state.right}")


def _cmd_lock() -> None:
    with DualArm.from_known_ports() as arms:
        arms.lock()
    print("locked: left, right")


def _cmd_unlock() -> None:
    with DualArm.from_known_ports() as arms:
        arms.unlock()
    print("unlocked: left, right")


def _format_position(state) -> str:
    if state is None:
        return "n/a"
    b, s, e, t = state.position
    return f"b={b:.6f} s={s:.6f} e={e:.6f} t={t:.6f}"


def _cmd_watch(rate_hz: float) -> None:
    period = 1.0 / rate_hz
    ports = find_arm_ports()
    print(f"left:  {ports.left}")
    print(f"right: {ports.right}")

    with DualArm.from_known_ports() as arms:
        try:
            while True:
                started = time.monotonic()
                state = arms.read_state()
                print(
                    f"left: {_format_position(state.left)} | "
                    f"right: {_format_position(state.right)}",
                    flush=True,
                )
                remaining = period - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            return


def main() -> None:
    parser = argparse.ArgumentParser(prog="roarm")
    subparsers = parser.add_subparsers(dest="command")
    subparsers.required = True

    subparsers.add_parser("status", help="print left/right port mapping and one state sample")
    subparsers.add_parser("lock", help="enable torque on both arms")
    subparsers.add_parser("unlock", help="disable torque on both arms")
    watch_parser = subparsers.add_parser("watch", help="continuously print left/right positions")
    watch_parser.add_argument(
        "--rate-hz",
        type=float,
        default=5.0,
        help="poll rate in Hz (default: 5.0)",
    )

    args = parser.parse_args()

    if args.command == "status":
        _cmd_status()
    elif args.command == "lock":
        _cmd_lock()
    elif args.command == "unlock":
        _cmd_unlock()
    elif args.command == "watch":
        _cmd_watch(args.rate_hz)


if __name__ == "__main__":
    main()
