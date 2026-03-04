"""ROS2 client node for a remote policy server."""

import argparse
import base64
import json
import socket
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from nodes._util import (
    JOINT_NAMES,
    publish_or_ignore_shutdown,
    run_node,
)

DEFAULT_POLICY_URL = "http://127.0.0.1:8000/predict"
DEFAULT_RATE_HZ = 5.0
DEFAULT_TIMEOUT_SEC = 1.0
WARNING_INTERVAL_SEC = 5.0


class PolicyClientNode(Node):
    def __init__(self, policy_url: str, rate_hz: float, timeout_sec: float):
        super().__init__("policy_client")
        self.policy_url = policy_url
        self.timeout_sec = timeout_sec

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.arm_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.status_pub = self.create_publisher(String, "/policy/status", 10)
        self.create_service(SetBool, "/policy_enable", self._srv_enable)
        self.create_subscription(
            CompressedImage, "/camera/image/compressed", self._on_image, sensor_qos,
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10,
        )

        self._state_lock = threading.Lock()
        self._last_frame: bytes | None = None
        self._last_joints: dict | None = None
        self._enabled = False
        self._generation = 0
        self._request_in_flight = False
        self._server_reachable: bool | None = None
        self._last_success_unix_sec: float | None = None
        self._last_error: str | None = None
        self._last_warning_monotonic = 0.0

        self.create_timer(1.0 / rate_hz, self._control_loop)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

        self.get_logger().info(f"Policy client ready ({self.policy_url})")

    def _on_image(self, msg: CompressedImage):
        with self._state_lock:
            self._last_frame = bytes(msg.data)

    def _on_joints(self, msg: JointState):
        with self._state_lock:
            self._last_joints = {
                "name": list(msg.name),
                "position": list(msg.position),
                "effort": list(msg.effort),
            }

    def _srv_enable(self, req, resp):
        with self._state_lock:
            self._enabled = bool(req.data)
            self._generation += 1
        self._publish_status()

        state = "enabled" if req.data else "disabled"
        self.get_logger().info(f"Policy client {state}")
        resp.success = True
        resp.message = f"Policy client {state}"
        return resp

    def _status_snapshot(self) -> dict:
        with self._state_lock:
            return {
                "enabled": self._enabled,
                "request_in_flight": self._request_in_flight,
                "server_reachable": self._server_reachable,
                "last_success_unix_sec": self._last_success_unix_sec,
                "last_error": self._last_error,
                "policy_url": self.policy_url,
            }

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(self._status_snapshot())
        publish_or_ignore_shutdown(self.status_pub, msg)

    def _control_loop(self):
        with self._state_lock:
            if not self._enabled or self._request_in_flight:
                return
            if self._last_frame is None or self._last_joints is None:
                return

            frame = self._last_frame
            joints = {
                "name": list(self._last_joints["name"]),
                "position": list(self._last_joints["position"]),
                "effort": list(self._last_joints["effort"]),
            }
            generation = self._generation
            self._request_in_flight = True

        thread = threading.Thread(
            target=self._request_policy,
            args=(frame, joints, generation),
            daemon=True,
        )
        thread.start()

    def _request_policy(self, frame: bytes, joints: dict, generation: int):
        try:
            action = self._fetch_action(frame, joints)
        except Exception as exc:
            self._record_request_failure(exc)
            return

        should_publish = False
        with self._state_lock:
            self._request_in_flight = False
            self._server_reachable = True
            self._last_success_unix_sec = time.time()
            self._last_error = None
            should_publish = self._enabled and self._generation == generation

        if should_publish:
            self._publish_action(action)

    def _fetch_action(self, frame: bytes, joints: dict) -> list[float]:
        payload = json.dumps({
            "image_jpeg_b64": base64.b64encode(frame).decode("ascii"),
            "joint_state": joints,
        }).encode("utf-8")

        request = urllib.request.Request(
            self.policy_url,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
            data = json.load(response)

        action = data.get("action")
        if not isinstance(action, list) or len(action) < 4:
            raise RuntimeError("Policy server returned an invalid action")
        return [float(value) for value in action[:4]]

    def _record_request_failure(self, exc: Exception):
        message = self._format_request_error(exc)
        with self._state_lock:
            self._request_in_flight = False
            self._server_reachable = False
            self._last_error = message

        now = time.monotonic()
        if now - self._last_warning_monotonic >= WARNING_INTERVAL_SEC:
            self._last_warning_monotonic = now
            self.get_logger().warning(f"Policy request failed: {message}")

    def _format_request_error(self, exc: Exception) -> str:
        if isinstance(exc, urllib.error.HTTPError):
            return f"HTTP {exc.code}"
        if isinstance(exc, urllib.error.URLError):
            reason = exc.reason
            if isinstance(reason, TimeoutError):
                return "Request timed out"
            if isinstance(reason, ConnectionRefusedError):
                return "Connection refused"
            if isinstance(reason, socket.gaierror):
                return "Host lookup failed"
            if isinstance(reason, OSError):
                if getattr(reason, "errno", None) == 111:
                    return "Connection refused"
                if getattr(reason, "errno", None) == 113:
                    return "No route to host"
            return "Server unreachable"
        if isinstance(exc, TimeoutError):
            return "Request timed out"
        return "Request failed"

    def _publish_action(self, action: list[float]):
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = action
        publish_or_ignore_shutdown(self.arm_pub, msg)


def main():
    parser = argparse.ArgumentParser(description="Run the remote policy client")
    parser.add_argument(
        "--url",
        default=DEFAULT_POLICY_URL,
        help=f"Policy server URL (default: {DEFAULT_POLICY_URL})",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_RATE_HZ,
        help=f"Policy polling rate in Hz (default: {DEFAULT_RATE_HZ})",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=DEFAULT_TIMEOUT_SEC,
        help=f"HTTP timeout in seconds (default: {DEFAULT_TIMEOUT_SEC})",
    )
    args = parser.parse_args()

    rclpy.init()
    run_node(PolicyClientNode(args.url, args.rate_hz, args.timeout_sec))


if __name__ == "__main__":
    main()
