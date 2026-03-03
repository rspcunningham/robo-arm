"""Web monitor for robot camera + joint telemetry.

Streams live camera view and joint data to any browser on the local network.
Uses daemon-thread spin so launch.py does NOT auto-detect this as a node.

Usage:
    uv run monitor --port 8080
"""

import argparse
import asyncio
import json
import math
import time

import rclpy
from aiohttp import web
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState

from nodes._util import JOINT_NAMES, spin_in_background

HTML_PAGE = """\
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Monitor</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: monospace; background: #111; color: #eee; padding: 12px; }
  h1 { font-size: 1.2em; margin-bottom: 8px; color: #8f8; }
  img { width: 100%; max-width: 640px; display: block; background: #000; }
  table { border-collapse: collapse; margin: 8px 0; }
  th, td { padding: 4px 12px; text-align: right; border: 1px solid #333; }
  th { color: #8cf; }
  td { font-variant-numeric: tabular-nums; }
  .label { text-align: left; color: #aaa; }
  .section { margin-top: 12px; font-size: 0.9em; color: #8cf; }
  .stale { color: #f88; }
</style>
</head>
<body>
<h1>Robot Monitor</h1>
<img id="cam" src="/stream" alt="camera">
<div class="section">Joint Positions (rad)</div>
<table id="pos-table">
  <tr><th class="label">Joint</th><th>Position</th><th>Degrees</th></tr>
</table>
<div class="section">Joint Efforts</div>
<table id="eff-table">
  <tr><th class="label">Joint</th><th>Effort</th></tr>
</table>
<div id="status" style="margin-top:12px;font-size:0.8em;color:#666"></div>
<script>
const JOINTS = %JOINTS%;
const posTable = document.getElementById('pos-table');
const effTable = document.getElementById('eff-table');
const status = document.getElementById('status');

// Build table rows
JOINTS.forEach(name => {
  let r = posTable.insertRow();
  r.innerHTML = `<td class="label">${name}</td><td id="pos-${name}">—</td><td id="deg-${name}">—</td>`;
  r = effTable.insertRow();
  r.innerHTML = `<td class="label">${name}</td><td id="eff-${name}">—</td>`;
});

const es = new EventSource('/events');
let lastEvent = 0;
es.onmessage = e => {
  const d = JSON.parse(e.data);
  lastEvent = Date.now();
  status.textContent = 'Connected';
  status.className = '';
  d.name.forEach((name, i) => {
    const pos = d.position[i];
    const eff = d.effort[i];
    document.getElementById('pos-' + name).textContent = pos.toFixed(4);
    document.getElementById('deg-' + name).textContent = (pos * 180 / Math.PI).toFixed(1);
    document.getElementById('eff-' + name).textContent = eff.toFixed(1);
  });
};
es.onerror = () => { status.textContent = 'Disconnected'; status.className = 'stale'; };
setInterval(() => {
  if (Date.now() - lastEvent > 2000) { status.textContent = 'Stale'; status.className = 'stale'; }
}, 1000);
</script>
</body>
</html>
""".replace("%JOINTS%", json.dumps(JOINT_NAMES))


class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor")
        self.last_jpeg: bytes | None = None
        self.last_joints: dict | None = None

        self.create_subscription(
            CompressedImage, "/camera/image/compressed", self._on_image, 10,
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10,
        )

    def _on_image(self, msg: CompressedImage):
        self.last_jpeg = bytes(msg.data)

    def _on_joints(self, msg: JointState):
        self.last_joints = {
            "name": list(msg.name),
            "position": list(msg.position),
            "effort": list(msg.effort),
        }


async def handle_index(request):
    return web.Response(text=HTML_PAGE, content_type="text/html")


async def handle_stream(request):
    node = request.app["node"]
    resp = web.StreamResponse()
    resp.content_type = "multipart/x-mixed-replace; boundary=frame"
    await resp.prepare(request)

    try:
        while True:
            jpeg = node.last_jpeg
            if jpeg is not None:
                await resp.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
            await asyncio.sleep(0.1)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


async def handle_events(request):
    node = request.app["node"]
    resp = web.StreamResponse()
    resp.content_type = "text/event-stream"
    resp.headers["Cache-Control"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    await resp.prepare(request)

    try:
        while True:
            joints = node.last_joints
            if joints is not None:
                payload = f"data: {json.dumps(joints)}\n\n"
                await resp.write(payload.encode())
            await asyncio.sleep(0.1)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


def main():
    parser = argparse.ArgumentParser(description="Web monitor for robot camera + telemetry")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    args = parser.parse_args()

    rclpy.init()
    node = MonitorNode()
    spin_in_background(node)

    app = web.Application()
    app["node"] = node
    app.router.add_get("/", handle_index)
    app.router.add_get("/stream", handle_stream)
    app.router.add_get("/events", handle_events)

    print(f"Monitor: http://0.0.0.0:{args.port}")
    try:
        web.run_app(app, host="0.0.0.0", port=args.port, print=None)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
