"""Web monitor for robot camera + joint telemetry.

Streams live camera view and joint data to any browser on the local network.
Uses daemon-thread spin so launch.py does NOT auto-detect this as a node.

Usage:
    uv run monitor --port 8080
"""

import argparse
import asyncio
import json

import rclpy
from aiohttp import web
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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
<img id="cam" alt="camera">
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

// Pull-based video: browser requests next frame only when current one is rendered
const cam = document.getElementById('cam');
let prevUrl = null;
async function streamVideo() {
  while (true) {
    try {
      const r = await fetch('/snapshot');
      if (!r.ok) { await new Promise(r => setTimeout(r, 200)); continue; }
      const blob = await r.blob();
      const url = URL.createObjectURL(blob);
      if (prevUrl) URL.revokeObjectURL(prevUrl);
      prevUrl = url;
      await new Promise((resolve, reject) => {
        cam.onload = resolve;
        cam.onerror = reject;
        cam.src = url;
      });
    } catch(e) {
      await new Promise(r => setTimeout(r, 1000));
    }
  }
}
streamVideo();

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
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("monitor")
        self._loop = loop
        self.joints_event = asyncio.Event()
        self.last_jpeg: bytes | None = None
        self.last_joints: dict | None = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage, "/camera/image/compressed", self._on_image, sensor_qos,
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
        self._loop.call_soon_threadsafe(self.joints_event.set)


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
            await node.joints_event.wait()
            node.joints_event.clear()
            joints = node.last_joints
            if joints is not None:
                payload = f"data: {json.dumps(joints)}\n\n"
                await resp.write(payload.encode())
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


async def _start_ros(app):
    loop = asyncio.get_running_loop()
    rclpy.init()
    node = MonitorNode(loop)
    spin_in_background(node)
    app["node"] = node
    node.get_logger().info("Monitor node ready")


async def _stop_ros(app):
    app["node"].destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


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

    web.run_app(app, host="0.0.0.0", port=args.port, print=None, shutdown_timeout=2.0)
