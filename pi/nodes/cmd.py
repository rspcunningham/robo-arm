"""Persistent robot command CLI.

Keeps ROS2 publishers/subscribers alive so DDS discovery happens once.
All subsequent commands execute instantly.

Usage:
    uv run cmd
    > move 0 0 1.57 0
    > snap
    > help
"""

import readline  # noqa: F401 — enables arrow keys / history in input()
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import SetBool, Trigger

from nodes._util import ArmCommander, spin_in_background

HELP = """\
Commands:
  move <base> <shoulder> <elbow> <hand> [spd=0]
  goto <base> <shoulder> <elbow> <hand> [force=500]
                                move and stop on contact
  home                          move to 0 0 1.57 1.57
  torque on|off
  stop                          emergency stop
  snap [filename.jpg]           save camera frame
  quit / Ctrl+C"""


class CmdNode(ArmCommander, Node):
    def __init__(self):
        super().__init__("cmd")
        # Arm
        self.arm_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.torque_cli = self.create_client(SetBool, "/torque_enable")
        self.estop_cli = self.create_client(Trigger, "/emergency_stop")
        # Joint feedback
        self._last_joints = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10,
        )
        # Camera
        self._last_frame = None
        self.create_subscription(
            CompressedImage, "/camera/image/compressed", self._on_image, 10,
        )

    def _on_joints(self, msg: JointState):
        self._last_joints = msg

    def _on_image(self, msg: CompressedImage):
        self._last_frame = msg


def main():
    rclpy.init()
    node = CmdNode()

    spin_in_background(node)

    print("Connecting...", end="", flush=True)
    while node.arm_pub.get_subscription_count() == 0:
        pass
    print(" ready.")
    print(HELP)

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0]

            if cmd == "quit":
                break

            elif cmd == "help":
                print(HELP)

            elif cmd == "home":
                node.publish_move([0.0, 0.0, 1.57, 1.57])
                print("OK")

            elif cmd == "move":
                if len(parts) < 5:
                    print("Usage: move <base> <shoulder> <elbow> <hand> [spd=0]")
                    continue
                try:
                    pos = [float(x) for x in parts[1:5]]
                except ValueError:
                    print("Joint values must be numbers")
                    continue
                spd = None
                for p in parts[5:]:
                    if p.startswith("spd="):
                        spd = float(p[4:])
                node.publish_move(pos, spd)
                print("OK")

            elif cmd == "goto":
                if len(parts) < 5:
                    print("Usage: goto <base> <shoulder> <elbow> <hand> [force=500]")
                    continue
                try:
                    pos = [float(x) for x in parts[1:5]]
                except ValueError:
                    print("Joint values must be numbers")
                    continue
                force_limit = 500.0
                for p in parts[5:]:
                    if p.startswith("force="):
                        force_limit = float(p[6:])
                node.publish_move(pos)
                print(f"goto {pos} force_limit={force_limit}")
                tolerance = 0.05
                try:
                    while True:
                        time.sleep(0.05)
                        js = node._last_joints
                        if js is None:
                            continue
                        # Check effort
                        if js.effort:
                            peak = max(abs(e) for e in js.effort)
                            if peak > force_limit:
                                node.emergency_stop()
                                idx = max(range(len(js.effort)),
                                          key=lambda i: abs(js.effort[i]))
                                name = js.name[idx] if idx < len(js.name) else str(idx)
                                print(f"Force limit hit on {name} "
                                      f"({abs(js.effort[idx]):.0f} > {force_limit:.0f})")
                                break
                        # Check position
                        if len(js.position) >= 4 and all(
                            abs(js.position[i] - pos[i]) < tolerance
                            for i in range(4)
                        ):
                            print("Reached target")
                            break
                except KeyboardInterrupt:
                    node.emergency_stop()
                    print("\nAborted")

            elif cmd == "torque":
                if len(parts) < 2:
                    print("Usage: torque on|off")
                    continue
                if not node.torque_cli.wait_for_service(timeout_sec=1.0):
                    print("Service not available")
                    continue
                req = SetBool.Request()
                req.data = parts[1] in ("on", "1", "true")
                future = node.torque_cli.call_async(req)
                future.result(timeout_sec=3.0)
                print("OK")

            elif cmd == "stop":
                if not node.estop_cli.wait_for_service(timeout_sec=1.0):
                    print("Service not available")
                    continue
                future = node.estop_cli.call_async(Trigger.Request())
                future.result(timeout_sec=3.0)
                print("Emergency stop sent")

            elif cmd == "snap":
                if node._last_frame is None:
                    print("No frame received yet")
                    continue
                path = parts[1] if len(parts) > 1 else "frame.jpg"
                data = bytes(node._last_frame.data)
                with open(path, "wb") as f:
                    f.write(data)
                print(f"Saved {path} ({len(data)} bytes)")

            else:
                print(f"Unknown command: {cmd}. Type 'help' for usage.")

    except KeyboardInterrupt:
        print()

    node.destroy_node()
    rclpy.shutdown()
