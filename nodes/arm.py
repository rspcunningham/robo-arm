"""RoArm-M2-S ROS2 driver node.

Publishes joint states at 20 Hz, accepts joint commands,
and exposes torque enable / emergency stop services.
"""

import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
import serial

JOINT_NAMES = ["base", "shoulder", "elbow", "hand"]


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm")

        # Persistent serial connection
        self.ser = serial.Serial(
            "/dev/ttyUSB0", 115200, timeout=0.05,
        )
        self.ser.rts = False
        self.ser.dtr = False

        # Silence debug output
        self._send({"T": 605, "cmd": 0})
        self.ser.reset_input_buffer()

        # Publisher
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        # Subscriber
        self.create_subscription(
            JointState, "/joint_commands", self._on_command, 10,
        )

        # Services
        self.create_service(SetBool, "/torque_enable", self._srv_torque)
        self.create_service(Trigger, "/emergency_stop", self._srv_estop)

        # Poll at 20 Hz
        self.create_timer(0.05, self._poll)

        self.get_logger().info("Arm node ready")

    # -- serial helpers ------------------------------------------------

    def _send(self, obj: dict):
        self.ser.write((json.dumps(obj) + "\n").encode())

    def _readline_json(self) -> dict | None:
        line = self.ser.readline()
        if not line:
            return None
        try:
            return json.loads(line)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None

    # -- callbacks -----------------------------------------------------

    def _poll(self):
        self._send({"T": 105})
        data = self._readline_json()
        if data is None or data.get("T") != 1051:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [
            -data["b"],            # base  = -b
            -data["s"],            # shoulder = -s
            data["e"],             # elbow = e
            math.pi - data["t"],   # hand  = pi - t
        ]
        msg.effort = [
            float(data.get("torB", 0)),
            float(data.get("torS", 0)),
            float(data.get("torE", 0)),
            float(data.get("torH", 0)),
        ]
        self.pub.publish(msg)

    def _on_command(self, msg: JointState):
        if len(msg.position) < 4:
            return
        base, shoulder, elbow, hand = msg.position[:4]
        cmd = {
            "T": 102,
            "base": -base,
            "shoulder": -shoulder,
            "elbow": elbow,
            "hand": math.pi - hand,
            "spd": 0,
            "acc": 0,
        }
        if msg.velocity:
            cmd["spd"] = msg.velocity[0]
        self._send(cmd)

    def _srv_torque(self, req, resp):
        self._send({"T": 210, "cmd": 1 if req.data else 0})
        resp.success = True
        return resp

    def _srv_estop(self, req, resp):
        self._send({"T": 0})
        resp.success = True
        resp.message = "Emergency stop sent"
        return resp

    # -- lifecycle -----------------------------------------------------

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
