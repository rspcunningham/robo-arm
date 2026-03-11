"""Reusable RoArm-M2-S ROS2 driver."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math

import serial
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from nodes._util import JOINT_NAMES, publish_or_ignore_shutdown

STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

DEFAULT_SERIAL_DEVICE = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "acff0192339bef11b2e7b69061ce3355-if00-port0"
)


def _normalize_namespace(value: str) -> str:
    value = value.strip()
    if not value or value == "/":
        return ""
    return value if value.startswith("/") else f"/{value}"


def _join_interface(namespace: str, leaf: str) -> str:
    namespace = _normalize_namespace(namespace)
    leaf = leaf.strip("/")
    if not namespace:
        return f"/{leaf}"
    return f"{namespace}/{leaf}"


@dataclass(frozen=True)
class ArmInterface:
    label: str
    serial_device: str
    namespace: str
    joint_states_topic: str
    joint_commands_topic: str
    light_status_topic: str
    torque_service: str
    light_service: str
    emergency_stop_service: str

    @classmethod
    def from_node(cls, node: Node) -> "ArmInterface":
        label = str(node.declare_parameter("label", "arm").value).strip() or "arm"
        serial_device = str(node.declare_parameter("serial_device", DEFAULT_SERIAL_DEVICE).value).strip()
        if not serial_device:
            raise ValueError("serial_device parameter must not be empty")

        namespace = _normalize_namespace(str(node.declare_parameter("interface_namespace", "").value))
        return cls(
            label=label,
            serial_device=serial_device,
            namespace=namespace,
            joint_states_topic=_join_interface(namespace, "joint_states"),
            joint_commands_topic=_join_interface(namespace, "joint_commands"),
            light_status_topic=_join_interface(namespace, "light/status"),
            torque_service=_join_interface(namespace, "torque_enable"),
            light_service=_join_interface(namespace, "light_enable"),
            emergency_stop_service=_join_interface(namespace, "emergency_stop"),
        )


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm")

        self.interface = ArmInterface.from_node(self)
        self._light_enabled = False

        self.ser = serial.Serial(
            self.interface.serial_device,
            115200,
            timeout=0.05,
        )
        self.ser.rts = False
        self.ser.dtr = False

        # Disable firmware debug chatter so JSON polling stays clean.
        self._send({"T": 605, "cmd": 0})
        self.ser.reset_input_buffer()

        self.joint_states_pub = self.create_publisher(
            JointState,
            self.interface.joint_states_topic,
            10,
        )
        self.light_status_pub = self.create_publisher(
            String,
            self.interface.light_status_topic,
            STATUS_QOS,
        )

        self.create_subscription(
            JointState,
            self.interface.joint_commands_topic,
            self._on_command,
            10,
        )

        self.create_service(SetBool, self.interface.torque_service, self._srv_torque)
        self.create_service(SetBool, self.interface.light_service, self._srv_light)
        self.create_service(Trigger, self.interface.emergency_stop_service, self._srv_estop)

        self.create_timer(0.05, self._poll)
        self._publish_light_status()

        self.get_logger().info(
            f"{self.interface.label} arm ready "
            f"(device={self.interface.serial_device}, namespace={self.interface.namespace or '/'})"
        )

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

    def _publish_light_status(self):
        msg = String()
        msg.data = json.dumps({"enabled": self._light_enabled})
        publish_or_ignore_shutdown(self.light_status_pub, msg)

    def _poll(self):
        self._send({"T": 105})
        data = self._readline_json()
        if data is None or data.get("T") != 1051:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [
            -data["b"],
            -data["s"],
            data["e"],
            math.pi - data["t"],
        ]
        msg.effort = [
            float(data.get("torB", 0)),
            float(data.get("torS", 0)),
            float(data.get("torE", 0)),
            float(data.get("torH", 0)),
        ]
        publish_or_ignore_shutdown(self.joint_states_pub, msg)

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

    def _srv_light(self, req, resp):
        self._send({"T": 114, "led": 255 if req.data else 0})
        self._light_enabled = bool(req.data)
        self._publish_light_status()
        state = "enabled" if req.data else "disabled"
        resp.success = True
        resp.message = f"Light {state}"
        return resp

    def _srv_estop(self, req, resp):
        self._send({"T": 0})
        resp.success = True
        resp.message = "Emergency stop sent"
        return resp

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()
