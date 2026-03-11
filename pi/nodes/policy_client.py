"""ROS2 client node for a remote OpenPI policy server."""

import argparse
import json
import socket
import threading
import time

import numpy as np

from openpi_client import image_tools
from openpi_client import websocket_client_policy
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from nodes._image import jpeg_bytes_to_rgb_array, ros_image_to_rgb_array
from nodes._util import (
    JOINT_NAMES,
    joint_positions_from_msg,
    publish_or_ignore_shutdown,
    run_node,
)

DEFAULT_POLICY_ENDPOINT = "ws://Robins-MacBook-Pro.local:8000"
DEFAULT_PROMPT = ""
MAX_RATE_HZ = 50.0
DEFAULT_RATE_HZ = 5.0
DEFAULT_TIMEOUT_SEC = 1.0
IMAGE_TOPIC = "/cam0/image_raw"
COMPRESSED_IMAGE_TOPIC = "/cam0/image/compressed"
WARNING_INTERVAL_SEC = 5.0
RESIZE_SIZE = 224
POLICY_CONFIG_TOPIC = "/policy/config"


class PolicyClientNode(Node):
    def __init__(self, policy_endpoint: str, rate_hz: float, timeout_sec: float, prompt: str):
        super().__init__("policy_client")
        self.policy_endpoint = _normalize_policy_endpoint(policy_endpoint)
        self.timeout_sec = timeout_sec
        self.prompt = prompt

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.arm_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.status_pub = self.create_publisher(String, "/policy/status", 10)
        self.create_service(SetBool, "/policy_enable", self._srv_enable)
        self.create_subscription(
            CompressedImage, COMPRESSED_IMAGE_TOPIC, self._on_compressed_image, sensor_qos,
        )
        self.create_subscription(
            Image, IMAGE_TOPIC, self._on_raw_image, sensor_qos,
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10,
        )
        self.create_subscription(
            String, POLICY_CONFIG_TOPIC, self._on_config, 10,
        )

        self._state_lock = threading.Lock()
        self._last_frame: np.ndarray | None = None
        self._last_joints: list[float] | None = None
        self._pending_actions: list[list[float]] = []
        self._enabled = False
        self._generation = 0
        self._request_in_flight = False
        self._server_reachable: bool | None = None
        self._last_success_unix_sec: float | None = None
        self._last_error: str | None = None
        self._server_metadata: dict | None = None
        self._client: websocket_client_policy.WebsocketClientPolicy | None = None
        self._last_warning_monotonic = 0.0
        self._last_image_warning_monotonic = 0.0
        self._last_joints_warning_monotonic = 0.0

        self.create_timer(1.0 / rate_hz, self._control_loop)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

        self.get_logger().info(f"Policy client ready ({self.policy_endpoint})")

    def _on_compressed_image(self, msg: CompressedImage):
        fmt = (msg.format or "").lower()
        if "jpeg" not in fmt and "jpg" not in fmt:
            now = time.monotonic()
            if now - self._last_image_warning_monotonic >= WARNING_INTERVAL_SEC:
                self._last_image_warning_monotonic = now
                self.get_logger().warning(
                    f"Dropping unsupported compressed frame format: {msg.format!r}"
                )
            return

        try:
            frame = jpeg_bytes_to_rgb_array(bytes(msg.data))
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_image_warning_monotonic >= WARNING_INTERVAL_SEC:
                self._last_image_warning_monotonic = now
                self.get_logger().warning(f"Dropping invalid compressed frame: {exc}")
            return

        with self._state_lock:
            self._last_frame = frame

    def _on_raw_image(self, msg: Image):
        try:
            frame = ros_image_to_rgb_array(msg)
        except ValueError as exc:
            now = time.monotonic()
            if now - self._last_image_warning_monotonic >= WARNING_INTERVAL_SEC:
                self._last_image_warning_monotonic = now
                self.get_logger().warning(f"Dropping unsupported camera frame: {exc}")
            return

        with self._state_lock:
            self._last_frame = frame

    def _on_joints(self, msg: JointState):
        joints = joint_positions_from_msg(msg)
        if joints is None:
            now = time.monotonic()
            if now - self._last_joints_warning_monotonic >= WARNING_INTERVAL_SEC:
                self._last_joints_warning_monotonic = now
                self.get_logger().warning("Dropping joint state with fewer than 4 positions")
            return

        with self._state_lock:
            self._last_joints = joints

    def _on_config(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring invalid policy config payload")
            return

        endpoint = payload.get("endpoint")
        prompt = payload.get("prompt")

        normalized_endpoint: str | None = None
        if endpoint is not None:
            try:
                normalized_endpoint = _normalize_policy_endpoint(str(endpoint))
            except ValueError as exc:
                self.get_logger().warning(f"Ignoring invalid policy endpoint: {exc}")
                return

        normalized_prompt: str | None = None
        if prompt is not None:
            normalized_prompt = str(prompt)

        with self._state_lock:
            endpoint_changed = normalized_endpoint is not None and normalized_endpoint != self.policy_endpoint
            prompt_changed = normalized_prompt is not None and normalized_prompt != self.prompt
            if not endpoint_changed and not prompt_changed:
                return

            if endpoint_changed:
                self._close_client_locked()
                self.policy_endpoint = normalized_endpoint
                self._server_reachable = None
                self._last_error = None
                self._server_metadata = None

            if prompt_changed:
                self.prompt = normalized_prompt

            self._pending_actions.clear()
            self._generation += 1

        if endpoint_changed and prompt_changed:
            self.get_logger().info(f"Policy endpoint updated to {normalized_endpoint}; prompt updated")
        elif endpoint_changed:
            self.get_logger().info(f"Policy endpoint updated to {normalized_endpoint}")
        else:
            self.get_logger().info("Policy prompt updated")
        self._publish_status()

    def _srv_enable(self, req, resp):
        with self._state_lock:
            self._enabled = bool(req.data)
            self._pending_actions.clear()
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
                "endpoint": self.policy_endpoint,
                "prompt": self.prompt,
                "server_metadata": self._server_metadata,
                "pending_action_count": len(self._pending_actions),
            }

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(self._status_snapshot())
        publish_or_ignore_shutdown(self.status_pub, msg)

    def _control_loop(self):
        action_to_publish: list[float] | None = None
        with self._state_lock:
            if not self._enabled:
                return

            if self._pending_actions:
                action_to_publish = self._pending_actions.pop(0)
            elif self._request_in_flight:
                return
            elif self._last_frame is None or self._last_joints is None:
                return
            else:
                frame = np.array(self._last_frame, copy=True)
                joints = list(self._last_joints)
                generation = self._generation
                self._request_in_flight = True

        if action_to_publish is not None:
            self._publish_action(action_to_publish)
            return

        thread = threading.Thread(
            target=self._request_policy,
            args=(frame, joints, generation),
            daemon=True,
        )
        thread.start()

    def _request_policy(self, frame: np.ndarray, joints: list[float], generation: int):
        try:
            action_chunk = self._fetch_action_chunk(frame, joints)
        except Exception as exc:
            self._record_request_failure(exc)
            return

        action_to_publish: list[float] | None = None
        with self._state_lock:
            self._request_in_flight = False
            self._server_reachable = True
            self._last_success_unix_sec = time.time()
            self._last_error = None
            if self._enabled and self._generation == generation and action_chunk:
                action_to_publish = action_chunk[0]
                self._pending_actions = action_chunk[1:]

        if action_to_publish is not None:
            self._publish_action(action_to_publish)

    def _fetch_action_chunk(self, frame: np.ndarray, joints: list[float]) -> list[list[float]]:
        client = self._get_client()
        frame = image_tools.convert_to_uint8(image_tools.resize_with_pad(frame, RESIZE_SIZE, RESIZE_SIZE))
        observation = {
            "observation/images/front": frame,
            "observation/state": np.asarray(joints, dtype=np.float32),
            "prompt": self.prompt,
        }
        data = client.infer(observation)
        action_chunk = _extract_action_chunk(data)
        if not action_chunk:
            raise RuntimeError("Policy server returned an empty action chunk")
        return action_chunk

    def _get_client(self) -> websocket_client_policy.WebsocketClientPolicy:
        with self._state_lock:
            if self._client is not None:
                return self._client
            endpoint = self.policy_endpoint

        client = websocket_client_policy.WebsocketClientPolicy(
            host=endpoint,
            port=None,
            connect_timeout_sec=self.timeout_sec,
            response_timeout_sec=self.timeout_sec,
        )
        metadata = client.get_server_metadata()

        with self._state_lock:
            if endpoint != self.policy_endpoint:
                client.close()
                if self._client is not None:
                    return self._client
            self._client = client
            self._server_metadata = metadata
            return client

    def _record_request_failure(self, exc: Exception):
        message = self._format_request_error(exc)
        with self._state_lock:
            self._close_client_locked()
            self._pending_actions.clear()
            self._request_in_flight = False
            self._server_reachable = False
            self._last_error = message

        now = time.monotonic()
        if now - self._last_warning_monotonic >= WARNING_INTERVAL_SEC:
            self._last_warning_monotonic = now
            self.get_logger().warning(f"Policy request failed: {message}")

    def _format_request_error(self, exc: Exception) -> str:
        if isinstance(exc, TimeoutError):
            return "Request timed out"
        if isinstance(exc, ConnectionRefusedError):
            return "Connection refused"
        if isinstance(exc, socket.gaierror):
            return "Host lookup failed"
        if isinstance(exc, OSError):
            if getattr(exc, "errno", None) == 111:
                return "Connection refused"
            if getattr(exc, "errno", None) == 113:
                return "No route to host"
            return "Server unreachable"
        return "Request failed"

    def _close_client_locked(self):
        client = self._client
        self._client = None
        self._server_metadata = None
        if client is not None:
            client.close()

    def _publish_action(self, action: list[float]):
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = action
        publish_or_ignore_shutdown(self.arm_pub, msg)


def main():
    parser = argparse.ArgumentParser(description="Run the remote OpenPI policy client")
    parser.add_argument(
        "--endpoint",
        default=DEFAULT_POLICY_ENDPOINT,
        help=f"Policy websocket endpoint (default: {DEFAULT_POLICY_ENDPOINT})",
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
        help=f"Connect/response timeout in seconds (default: {DEFAULT_TIMEOUT_SEC})",
    )
    parser.add_argument(
        "--prompt",
        default=DEFAULT_PROMPT,
        help="Prompt sent with each policy request (default: empty string)",
    )
    args = parser.parse_args()
    if args.rate_hz <= 0 or args.rate_hz > MAX_RATE_HZ:
        parser.error(f"--rate-hz must be in (0, {MAX_RATE_HZ}]")

    rclpy.init()
    run_node(PolicyClientNode(args.endpoint, args.rate_hz, args.timeout_sec, args.prompt))


def _normalize_policy_endpoint(endpoint: str) -> str:
    endpoint = endpoint.strip()
    if not endpoint:
        raise ValueError("endpoint is empty")
    if endpoint.startswith("ws://") or endpoint.startswith("wss://"):
        return endpoint
    if endpoint.startswith("http://"):
        return "ws://" + endpoint.removeprefix("http://")
    if endpoint.startswith("https://"):
        return "wss://" + endpoint.removeprefix("https://")
    return f"ws://{endpoint}"


def _extract_action_chunk(response: dict) -> list[list[float]]:
    actions = response.get("actions")
    if actions is None:
        raise RuntimeError("Policy server did not return actions")

    action_array = np.asarray(actions, dtype=np.float32)
    if action_array.ndim == 0:
        raise RuntimeError("Policy server returned a scalar action")
    if action_array.ndim == 1:
        action_array = action_array.reshape(1, -1)
    if action_array.ndim != 2:
        raise RuntimeError(f"Policy server returned invalid action shape: {tuple(action_array.shape)}")
    if action_array.shape[1] != len(JOINT_NAMES):
        raise RuntimeError(
            f"Policy server returned {action_array.shape[1]} action values, expected {len(JOINT_NAMES)}"
        )
    return [[float(value) for value in row.tolist()] for row in action_array]


if __name__ == "__main__":
    main()
