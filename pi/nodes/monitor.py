"""Web monitor for robot camera, telemetry, and control state.

Streams live camera view and joint data to any browser on the local network.
Uses daemon-thread spin so launch.py does NOT auto-detect this as a node.

Usage:
    uv run monitor --port 8080
"""

import argparse
import asyncio
import json
from pathlib import Path

import rclpy
from aiohttp import web
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image as RosImage, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from nodes._image import ros_image_to_jpeg_bytes
from nodes._util import (
    JOINT_NAMES,
    publish_or_ignore_shutdown,
    shutdown_background_node,
    spin_in_background,
    wait_for_future,
)

IMAGE_TOPIC = "/cam0/image_raw"
COMPRESSED_IMAGE_TOPIC = "/cam0/image/compressed"

HTML_TEMPLATE = (Path(__file__).with_name("monitor.html")).read_text(encoding="utf-8")
HTML_PAGE = HTML_TEMPLATE.replace("%JOINTS%", json.dumps(JOINT_NAMES))


class MonitorNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("monitor")
        self._loop = loop
        self.state_event = asyncio.Event()
        self.last_jpeg: bytes | None = None
        self._last_image_warning_monotonic = 0.0
        self.last_joints: dict | None = None
        self.last_control_status = {
            "mode": "idle",
            "teleop_enabled": False,
            "torque_enabled": True,
        }
        self.last_light_status = {
            "enabled": False,
        }
        self.last_policy_status = {
            "enabled": False,
            "request_in_flight": False,
            "server_reachable": None,
            "last_success_unix_sec": None,
            "last_error": None,
            "endpoint": "",
            "prompt": "",
            "server_metadata": None,
            "pending_action_count": 0,
        }
        self.last_record_status = {
            "state": "idle",
            "busy": False,
            "busy_action": None,
            "status_message": "Idle",
            "session_active": False,
            "episode_active": False,
            "task": "",
            "repo_id": "",
            "fps": None,
            "episodes_saved": 0,
            "current_episode_frames": 0,
            "teleop_ready": False,
            "last_error": None,
        }
        self.control_policy_cli = self.create_client(SetBool, "/control/set_policy_active")
        self.control_teleop_cli = self.create_client(SetBool, "/control/set_teleop_active")
        self.control_torque_cli = self.create_client(SetBool, "/control/set_manual_torque")
        self.light_cli = self.create_client(SetBool, "/light_enable")
        self.policy_config_pub = self.create_publisher(String, "/policy/config", 10)
        self.record_command_pub = self.create_publisher(String, "/record/command", 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage, COMPRESSED_IMAGE_TOPIC, self._on_compressed_image, sensor_qos,
        )
        self.create_subscription(
            RosImage, IMAGE_TOPIC, self._on_image, sensor_qos,
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10,
        )
        self.create_subscription(
            String, "/control/status", self._on_control_status, control_qos,
        )
        self.create_subscription(
            String, "/light/status", self._on_light_status, control_qos,
        )
        self.create_subscription(
            String, "/policy/status", self._on_policy_status, 10,
        )
        self.create_subscription(
            String, "/record/status", self._on_record_status, control_qos,
        )

    def _notify(self):
        self._loop.call_soon_threadsafe(self.state_event.set)

    def _state_payload(self) -> dict:
        return {
            "joints": self.last_joints,
            "control": self.last_control_status,
            "light": self.last_light_status,
            "policy": self.last_policy_status,
            "record": self.last_record_status,
        }

    def _on_compressed_image(self, msg: CompressedImage):
        fmt = (msg.format or "").lower()
        if "jpeg" not in fmt and "jpg" not in fmt:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_image_warning_monotonic >= 5.0:
                self._last_image_warning_monotonic = now
                self.get_logger().warning(
                    f"Dropping unsupported compressed camera frame format: {msg.format!r}"
                )
            return
        self.last_jpeg = bytes(msg.data)

    def _on_image(self, msg: RosImage):
        try:
            self.last_jpeg = ros_image_to_jpeg_bytes(msg, quality=80)
        except ValueError as exc:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_image_warning_monotonic >= 5.0:
                self._last_image_warning_monotonic = now
                self.get_logger().warning(f"Dropping unsupported camera frame: {exc}")

    def _on_joints(self, msg: JointState):
        self.last_joints = {
            "name": list(msg.name),
            "position": list(msg.position),
            "effort": list(msg.effort),
        }
        self._notify()

    def _on_control_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_control_status = {
            "mode": payload.get("mode", "idle"),
            "teleop_enabled": bool(payload.get("teleop_enabled", False)),
            "torque_enabled": bool(payload.get("torque_enabled", True)),
        }
        self._notify()

    def _on_light_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_light_status = {
            "enabled": bool(payload.get("enabled", False)),
        }
        self._notify()

    def _on_policy_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_policy_status = {
            "enabled": bool(payload.get("enabled", False)),
            "request_in_flight": bool(payload.get("request_in_flight", False)),
            "server_reachable": payload.get("server_reachable"),
            "last_success_unix_sec": payload.get("last_success_unix_sec"),
            "last_error": payload.get("last_error"),
            "endpoint": str(payload.get("endpoint", "")),
            "prompt": str(payload.get("prompt", "")),
            "server_metadata": payload.get("server_metadata"),
            "pending_action_count": int(payload.get("pending_action_count", 0)),
        }
        self._notify()

    def _on_record_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_record_status = {
            "state": payload.get("state", "idle"),
            "busy": bool(payload.get("busy", False)),
            "busy_action": payload.get("busy_action"),
            "status_message": payload.get("status_message", "Idle"),
            "session_active": bool(payload.get("session_active", False)),
            "episode_active": bool(payload.get("episode_active", False)),
            "task": payload.get("task", ""),
            "repo_id": payload.get("repo_id", ""),
            "fps": payload.get("fps"),
            "episodes_saved": int(payload.get("episodes_saved", 0)),
            "current_episode_frames": int(payload.get("current_episode_frames", 0)),
            "teleop_ready": bool(payload.get("teleop_ready", False)),
            "last_error": payload.get("last_error"),
        }
        self._notify()


async def handle_index(request):
    return web.Response(text=HTML_PAGE, content_type="text/html")


async def handle_snapshot(request):
    node = request.app["node"]
    jpeg = node.last_jpeg
    if jpeg is None:
        return web.Response(status=503, text="No frame yet")
    return web.Response(body=jpeg, content_type="image/jpeg")


async def handle_events(request):
    node = request.app["node"]
    resp = web.StreamResponse()
    resp.content_type = "text/event-stream"
    resp.headers["Cache-Control"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    await resp.prepare(request)

    try:
        while True:
            await node.state_event.wait()
            node.state_event.clear()
            payload = f"data: {json.dumps(node._state_payload())}\n\n"
            await resp.write(payload.encode())
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


async def _set_service_bool(request, client_attr: str, enabled: bool, unavailable_error: str, default_error: str):
    node = request.app["node"]
    client = getattr(node, client_attr)
    if not await asyncio.to_thread(client.wait_for_service, 1.0):
        return web.json_response(
            {"error": unavailable_error},
            status=503,
        )

    req = SetBool.Request()
    req.data = enabled
    future = client.call_async(req)

    try:
        result = await asyncio.to_thread(wait_for_future, future, 2.0)
    except TimeoutError as exc:
        return web.json_response({"error": str(exc)}, status=504)
    except Exception as exc:
        return web.json_response({"error": str(exc)}, status=500)

    if not result.success:
        return web.json_response({"error": result.message or default_error}, status=500)

    return web.json_response({"ok": True, "enabled": enabled})


async def _publish_record_command(request, payload: dict, unavailable_error: str):
    node = request.app["node"]
    publisher = node.record_command_pub
    if publisher.get_subscription_count() <= 0:
        return web.json_response({"error": unavailable_error}, status=503)

    msg = String()
    msg.data = json.dumps(payload)
    publish_or_ignore_shutdown(publisher, msg)
    return web.json_response({"ok": True})


async def _publish_policy_config(request, payload: dict):
    node = request.app["node"]
    publisher = node.policy_config_pub
    if publisher.get_subscription_count() <= 0:
        return web.json_response({"error": "Policy client is unavailable"}, status=503)

    msg = String()
    msg.data = json.dumps(payload)
    publish_or_ignore_shutdown(publisher, msg)
    return web.json_response({"ok": True, **payload})


async def handle_control_policy_enable(request):
    return await _set_service_bool(
        request,
        "control_policy_cli",
        True,
        "Control manager policy service unavailable",
        "Policy toggle failed",
    )


async def handle_control_policy_disable(request):
    return await _set_service_bool(
        request,
        "control_policy_cli",
        False,
        "Control manager policy service unavailable",
        "Policy toggle failed",
    )


async def handle_control_teleop_enable(request):
    return await _set_service_bool(
        request,
        "control_teleop_cli",
        True,
        "Control manager teleop service unavailable",
        "Teleop toggle failed",
    )


async def handle_control_teleop_disable(request):
    return await _set_service_bool(
        request,
        "control_teleop_cli",
        False,
        "Control manager teleop service unavailable",
        "Teleop toggle failed",
    )


async def handle_control_torque_enable(request):
    return await _set_service_bool(
        request,
        "control_torque_cli",
        True,
        "Control manager torque service unavailable",
        "Torque toggle failed",
    )


async def handle_control_torque_disable(request):
    return await _set_service_bool(
        request,
        "control_torque_cli",
        False,
        "Control manager torque service unavailable",
        "Torque toggle failed",
    )


async def handle_light_enable(request):
    return await _set_service_bool(
        request,
        "light_cli",
        True,
        "Light service unavailable",
        "Light toggle failed",
    )


async def handle_light_disable(request):
    return await _set_service_bool(
        request,
        "light_cli",
        False,
        "Light service unavailable",
        "Light toggle failed",
    )


async def handle_record_begin_session(request):
    try:
        payload = await request.json()
    except Exception:
        return web.json_response({"error": "Invalid JSON payload"}, status=400)

    repo_id = str(payload.get("repo_id", "")).strip()
    task = str(payload.get("task", "")).strip()
    try:
        fps = int(payload.get("fps", 10))
    except (TypeError, ValueError):
        return web.json_response({"error": "FPS must be a valid integer"}, status=400)
    public = bool(payload.get("public", False))

    if not repo_id:
        return web.json_response({"error": "Repository id is required"}, status=400)
    if not task:
        return web.json_response({"error": "Task is required"}, status=400)
    if fps <= 0:
        return web.json_response({"error": "FPS must be positive"}, status=400)

    return await _publish_record_command(
        request,
        {
            "action": "begin_session",
            "repo_id": repo_id,
            "task": task,
            "fps": fps,
            "public": public,
        },
        "Record manager is unavailable",
    )


async def handle_record_start(request):
    return await _publish_record_command(
        request,
        {"action": "start"},
        "Record manager is unavailable",
    )


async def handle_record_done(request):
    return await _publish_record_command(
        request,
        {"action": "done"},
        "Record manager is unavailable",
    )


async def handle_record_finish(request):
    return await _publish_record_command(
        request,
        {"action": "finish"},
        "Record manager is unavailable",
    )


async def handle_policy_config(request):
    try:
        payload = await request.json()
    except Exception:
        return web.json_response({"error": "Invalid JSON payload"}, status=400)

    config: dict[str, str] = {}
    if "endpoint" in payload:
        endpoint = str(payload.get("endpoint", "")).strip()
        if not endpoint:
            return web.json_response({"error": "Policy endpoint must be non-empty"}, status=400)
        config["endpoint"] = endpoint

    if "prompt" in payload:
        config["prompt"] = str(payload.get("prompt", ""))

    if not config:
        return web.json_response({"error": "At least one policy config field is required"}, status=400)

    return await _publish_policy_config(request, config)


async def _start_ros(app):
    loop = asyncio.get_running_loop()
    rclpy.init()
    node = MonitorNode(loop)
    app["spin_thread"] = spin_in_background(node)
    app["node"] = node
    node.get_logger().info("Monitor node ready")


async def _stop_ros(app):
    shutdown_background_node(app["node"], app["spin_thread"])


def main():
    parser = argparse.ArgumentParser(description="Web monitor for robot camera + telemetry")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    args = parser.parse_args()

    app = web.Application()
    app.on_startup.append(_start_ros)
    app.on_cleanup.append(_stop_ros)
    app.router.add_get("/", handle_index)
    app.router.add_get("/snapshot", handle_snapshot)
    app.router.add_get("/events", handle_events)
    app.router.add_post("/api/control/policy/enable", handle_control_policy_enable)
    app.router.add_post("/api/control/policy/disable", handle_control_policy_disable)
    app.router.add_post("/api/control/teleop/enable", handle_control_teleop_enable)
    app.router.add_post("/api/control/teleop/disable", handle_control_teleop_disable)
    app.router.add_post("/api/control/torque/enable", handle_control_torque_enable)
    app.router.add_post("/api/control/torque/disable", handle_control_torque_disable)
    app.router.add_post("/api/light/enable", handle_light_enable)
    app.router.add_post("/api/light/disable", handle_light_disable)
    app.router.add_post("/api/policy/config", handle_policy_config)
    app.router.add_post("/api/record/begin-session", handle_record_begin_session)
    app.router.add_post("/api/record/start", handle_record_start)
    app.router.add_post("/api/record/done", handle_record_done)
    app.router.add_post("/api/record/finish", handle_record_finish)

    web.run_app(
        app,
        host="0.0.0.0",
        port=args.port,
        print=None,
        shutdown_timeout=0.5,
        handle_signals=False,
    )
