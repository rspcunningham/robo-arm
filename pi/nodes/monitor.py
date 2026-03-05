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
from sensor_msgs.msg import Image as RosImage, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from nodes._image import ros_image_to_jpeg_bytes
from nodes._util import (
    JOINT_NAMES,
    shutdown_background_node,
    spin_in_background,
    wait_for_future,
)

IMAGE_TOPIC = "/cam0/image_raw"

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
            "torque_enabled": True,
        }
        self.last_light_status = {
            "enabled": False,
        }
        self.control_policy_cli = self.create_client(SetBool, "/control/set_policy_active")
        self.control_torque_cli = self.create_client(SetBool, "/control/set_manual_torque")
        self.light_cli = self.create_client(SetBool, "/light_enable")

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

    def _notify(self):
        self._loop.call_soon_threadsafe(self.state_event.set)

    def _state_payload(self) -> dict:
        return {
            "joints": self.last_joints,
            "control": self.last_control_status,
            "light": self.last_light_status,
        }

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
    app.router.add_post("/api/control/torque/enable", handle_control_torque_enable)
    app.router.add_post("/api/control/torque/disable", handle_control_torque_disable)
    app.router.add_post("/api/light/enable", handle_light_enable)
    app.router.add_post("/api/light/disable", handle_light_disable)

    web.run_app(
        app,
        host="0.0.0.0",
        port=args.port,
        print=None,
        shutdown_timeout=0.5,
        handle_signals=False,
    )
