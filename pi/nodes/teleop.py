"""Low-latency teleop bridge from leader arm state to follower arm commands."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from nodes._util import joint_positions_from_msg, publish_or_ignore_shutdown, run_node

STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
JOINT_COUNT = 4
WARNING_INTERVAL_SEC = 5.0


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop")

        self._leader_topic = str(
            self.declare_parameter("leader_joint_states_topic", "/leader/joint_states").value
        ).strip()
        self._follower_topic = str(
            self.declare_parameter("follower_joint_commands_topic", "/joint_commands").value
        ).strip()
        self._scale = self._float_list_param("scale", [1.0] * JOINT_COUNT)
        self._offset = self._float_list_param("offset", [0.0] * JOINT_COUNT)
        self._invert = self._bool_list_param("invert", [False] * JOINT_COUNT)

        self._mode = "idle"
        self._outputs_converged = False
        self._last_leader_state_monotonic: float | None = None
        self._forwarded_count = 0
        self._dropped_count = 0
        self._last_warning_monotonic = 0.0
        self._last_status_json: str | None = None

        self.command_pub = self.create_publisher(JointState, self._follower_topic, COMMAND_QOS)
        self.status_pub = self.create_publisher(String, "/teleop/status", STATUS_QOS)

        self.create_subscription(
            JointState,
            self._leader_topic,
            self._on_leader_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/control/status",
            self._on_control_status,
            STATUS_QOS,
        )

        self.create_timer(1.0, self._publish_status)
        self._publish_status()

        self.get_logger().info(
            f"Teleop ready (leader={self._leader_topic}, follower={self._follower_topic})"
        )

    def _float_list_param(self, name: str, default: list[float]) -> tuple[float, ...]:
        values = [float(value) for value in self.declare_parameter(name, default).value]
        if len(values) != JOINT_COUNT:
            raise ValueError(f"{name} must contain exactly {JOINT_COUNT} values")
        return tuple(values)

    def _bool_list_param(self, name: str, default: list[bool]) -> tuple[bool, ...]:
        values = [bool(value) for value in self.declare_parameter(name, default).value]
        if len(values) != JOINT_COUNT:
            raise ValueError(f"{name} must contain exactly {JOINT_COUNT} values")
        return tuple(values)

    def _teleop_active(self) -> bool:
        return self._mode == "teleop" and self._outputs_converged

    def _status_snapshot(self) -> dict:
        leader_fresh = False
        leader_age_sec = None
        if self._last_leader_state_monotonic is not None:
            leader_age_sec = max(0.0, time.monotonic() - self._last_leader_state_monotonic)
            leader_fresh = leader_age_sec <= 1.0
        return {
            "enabled": self._teleop_active(),
            "mode": self._mode,
            "outputs_converged": self._outputs_converged,
            "leader_topic": self._leader_topic,
            "follower_topic": self._follower_topic,
            "leader_fresh": leader_fresh,
            "leader_age_sec": leader_age_sec,
            "forwarded_count": self._forwarded_count,
            "dropped_count": self._dropped_count,
        }

    def _publish_status(self):
        payload = json.dumps(self._status_snapshot())
        if payload == self._last_status_json:
            return
        self._last_status_json = payload
        msg = String()
        msg.data = payload
        publish_or_ignore_shutdown(self.status_pub, msg)

    def _on_control_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        mode = str(payload.get("mode", "idle"))
        outputs_converged = bool(payload.get("outputs_converged", False))
        if mode == self._mode and outputs_converged == self._outputs_converged:
            return

        self._mode = mode
        self._outputs_converged = outputs_converged
        self._publish_status()

    def _on_leader_joints(self, msg: JointState):
        self._last_leader_state_monotonic = time.monotonic()

        if not self._teleop_active():
            return

        joints = joint_positions_from_msg(msg)
        if joints is None:
            self._dropped_count += 1
            now = time.monotonic()
            if now - self._last_warning_monotonic >= WARNING_INTERVAL_SEC:
                self._last_warning_monotonic = now
                self.get_logger().warning("Dropping leader joint state with fewer than 4 positions")
            self._publish_status()
            return

        out = JointState()
        out.position = [
            (-value if invert else value) * scale + offset
            for value, invert, scale, offset in zip(
                joints,
                self._invert,
                self._scale,
                self._offset,
                strict=True,
            )
        ]
        if msg.velocity:
            out.velocity = [float(msg.velocity[0])]
        publish_or_ignore_shutdown(self.command_pub, out)
        self._forwarded_count += 1


def main():
    rclpy.init()
    run_node(TeleopNode())
