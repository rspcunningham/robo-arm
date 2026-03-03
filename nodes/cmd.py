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
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import SetBool, Trigger

HELP = """\
Commands:
  move <base> <shoulder> <elbow> <hand> [spd=0]
  home                          move to 0 0 1.57 1.57
  torque on|off
  stop                          emergency stop
  snap [filename.jpg]           save camera frame
  quit / Ctrl+C"""


class CmdNode(Node):
    def __init__(self):
        super().__init__("cmd")
        # Arm
        self.arm_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.torque_cli = self.create_client(SetBool, "/torque_enable")
        self.estop_cli = self.create_client(Trigger, "/emergency_stop")
        # Camera
        self._last_frame = None
        self.create_subscription(
            CompressedImage, "/camera/image/compressed", self._on_image, 10,
        )

    def _on_image(self, msg: CompressedImage):
        self._last_frame = msg

    def publish_move(self, pos: list[float], spd: float = 100):
        msg = JointState()
        msg.name = ["base", "shoulder", "elbow", "hand"]
        msg.position = pos
        msg.velocity = [spd]
        self.arm_pub.publish(msg)


def main():
    rclpy.init()
    node = CmdNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

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
                spd = 100
                for p in parts[5:]:
                    if p.startswith("spd="):
                        spd = float(p[4:])
                node.publish_move(pos, spd)
                print("OK")

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
