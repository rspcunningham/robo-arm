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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from nodes._util import JOINT_NAMES, shutdown_background_node, spin_in_background

HTML_PAGE = """\
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Monitor</title>
<style>
  :root {
    --bg: #0d1117;
    --panel: #151b23;
    --panel-2: #10161d;
    --line: #27313d;
    --line-soft: #1e2630;
    --text: #e6edf3;
    --muted: #95a2b3;
    --accent: #3fb950;
    --warn: #d29922;
    --danger: #f85149;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    min-height: 100vh;
    padding: 16px;
    color: var(--text);
    background: var(--bg);
    font-family: "SF Pro Text", "Segoe UI", "Helvetica Neue", sans-serif;
  }
  .shell {
    max-width: 1600px;
    margin: 0 auto;
  }
  .topbar {
    display: flex;
    flex-wrap: nowrap;
    justify-content: space-between;
    gap: 12px;
    align-items: center;
    margin-bottom: 14px;
  }
  .eyebrow {
    font: 600 0.74rem/1 "SFMono-Regular", "Menlo", monospace;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: var(--muted);
    margin-bottom: 6px;
  }
  h1 {
    font-size: clamp(1.15rem, 1.8vw, 1.5rem);
    line-height: 1.1;
    font-weight: 650;
  }
  .subhead {
    margin-top: 4px;
    color: var(--muted);
    font-size: 0.88rem;
    line-height: 1.35;
  }
  .status-pill {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    padding: 8px 12px;
    border-radius: 999px;
    border: 1px solid var(--line);
    background: var(--panel);
    font: 600 0.75rem/1 "SFMono-Regular", "Menlo", monospace;
    letter-spacing: 0.04em;
    text-transform: uppercase;
    white-space: nowrap;
  }
  .status-pill::before {
    content: "";
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: var(--warn);
  }
  .status-pill.online::before {
    background: var(--accent);
  }
  .status-pill.stale::before {
    background: var(--danger);
  }
  .layout {
    display: grid;
    grid-template-columns: minmax(0, 2.35fr) minmax(300px, 0.85fr);
    gap: 14px;
    align-items: start;
  }
  .panel {
    border: 1px solid var(--line);
    border-radius: 12px;
    padding: 14px;
    background: var(--panel);
  }
  .panel-title {
    color: var(--muted);
    font: 600 0.72rem/1 "SFMono-Regular", "Menlo", monospace;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    margin-bottom: 10px;
  }
  .main-panel {
    padding: 12px;
  }
  .frame-meta {
    margin-bottom: 10px;
    display: flex;
    justify-content: space-between;
    gap: 12px;
    align-items: baseline;
  }
  .frame-meta strong {
    font-size: 0.98rem;
    font-weight: 600;
  }
  .frame-meta span {
    color: var(--muted);
    font: 500 0.78rem/1.2 "SFMono-Regular", "Menlo", monospace;
  }
  .camera-frame {
    overflow: hidden;
    border-radius: 10px;
    border: 1px solid var(--line-soft);
    background: #05080d;
    min-height: 420px;
  }
  .camera-frame img {
    display: block;
    width: 100%;
    aspect-ratio: 16 / 10;
    object-fit: contain;
    background: #05080d;
  }
  .frame-caption {
    margin-top: 10px;
    display: flex;
    justify-content: space-between;
    gap: 12px;
    color: var(--muted);
    font: 500 0.78rem/1.4 "SFMono-Regular", "Menlo", monospace;
  }
  .tables {
    margin-top: 14px;
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 14px;
  }
  .table-panel {
    border: 1px solid var(--line-soft);
    border-radius: 10px;
    padding: 12px;
    background: var(--panel-2);
  }
  .table-title {
    margin-bottom: 10px;
    color: var(--muted);
    font: 600 0.72rem/1 "SFMono-Regular", "Menlo", monospace;
    letter-spacing: 0.08em;
    text-transform: uppercase;
  }
  table {
    width: 100%;
    border-collapse: collapse;
  }
  th, td {
    padding: 8px 0;
    border-bottom: 1px solid var(--line-soft);
    text-align: right;
    font-variant-numeric: tabular-nums;
  }
  tr:last-child td {
    border-bottom: 0;
  }
  th {
    color: var(--muted);
    font: 600 0.68rem/1 "SFMono-Regular", "Menlo", monospace;
    letter-spacing: 0.06em;
    text-transform: uppercase;
  }
  td {
    font-size: 0.9rem;
  }
  .label {
    text-align: left;
    color: var(--text);
  }
  .summary-panel {
    display: grid;
    gap: 10px;
  }
  .summary-group {
    border: 1px solid var(--line-soft);
    border-radius: 10px;
    padding: 12px;
    background: var(--panel-2);
  }
  .summary-row {
    display: flex;
    align-items: baseline;
    justify-content: space-between;
    gap: 10px;
  }
  .summary-label {
    color: var(--muted);
    font: 600 0.68rem/1 "SFMono-Regular", "Menlo", monospace;
    letter-spacing: 0.06em;
    text-transform: uppercase;
  }
  .summary-value {
    font-size: 1.05rem;
    font-weight: 650;
    line-height: 1.1;
    text-align: right;
    font-variant-numeric: tabular-nums;
  }
  .summary-meta {
    margin-top: 6px;
    color: var(--muted);
    font-size: 0.8rem;
    line-height: 1.35;
  }
  .footer {
    margin-top: 2px;
    color: var(--muted);
    font: 500 0.76rem/1.4 "SFMono-Regular", "Menlo", monospace;
  }
  @media (max-width: 980px) {
    .layout, .tables { grid-template-columns: 1fr; }
    .camera-frame { min-height: 320px; }
    .topbar {
      flex-wrap: wrap;
      align-items: flex-start;
    }
    .status-pill {
      width: fit-content;
    }
  }
  @media (max-width: 720px) {
    .frame-meta,
    .frame-caption {
      flex-direction: column;
      align-items: flex-start;
    }
  }
  @media (max-width: 640px) {
    body { padding: 10px; }
    .panel, .table-panel { padding: 10px; }
    .camera-frame { min-height: 240px; }
  }
</style>
</head>
<body>
<div class="shell">
  <section class="topbar">
    <div>
      <div class="eyebrow">Pi Telemetry</div>
      <h1>Robot Monitor</h1>
      <p class="subhead">Live camera and joint telemetry for bench inspection.</p>
    </div>
    <div id="status-pill" class="status-pill">Awaiting stream</div>
  </section>

  <section class="layout">
    <div class="panel main-panel">
      <div class="frame-meta">
        <strong>Camera Feed</strong>
        <span id="updated-at">No telemetry yet</span>
      </div>
      <div class="camera-frame">
        <img id="cam" alt="camera">
      </div>
      <div class="frame-caption">
        <span id="cam-status">Camera feed pending</span>
        <span>Snapshot stream</span>
      </div>

      <div class="tables">
        <div class="table-panel">
          <div class="table-title">Joint Positions</div>
          <table id="pos-table">
            <tr><th class="label">Joint</th><th>Radians</th><th>Degrees</th></tr>
          </table>
        </div>
        <div class="table-panel">
          <div class="table-title">Joint Effort</div>
          <table id="eff-table">
            <tr><th class="label">Joint</th><th>Value</th></tr>
          </table>
        </div>
      </div>
    </div>

    <aside class="panel summary-panel">
      <div class="panel-title">Telemetry Summary</div>

      <div class="summary-group">
        <div class="summary-row">
          <div class="summary-label">Stream</div>
          <div class="summary-value" id="stream-state">Idle</div>
        </div>
        <div class="summary-meta" id="stream-meta">Waiting for telemetry</div>
      </div>

      <div class="summary-group">
        <div class="summary-row">
          <div class="summary-label">SPI Recoveries / Min</div>
          <div class="summary-value" id="spi-rate">0</div>
        </div>
        <div class="summary-meta" id="spi-rate-meta">No link resets in the last minute</div>
      </div>

      <div class="summary-group">
        <div class="summary-row">
          <div class="summary-label">Total Recoveries</div>
          <div class="summary-value" id="spi-total">0</div>
        </div>
        <div class="summary-meta">Since the current camera process started</div>
      </div>

      <div class="summary-group">
        <div class="summary-row">
          <div class="summary-label">Last Recovery</div>
          <div class="summary-value" id="spi-last">Never</div>
        </div>
        <div class="summary-meta" id="spi-last-meta">SPI link has not reset yet</div>
      </div>

      <div class="footer" id="footer-status">Event stream disconnected</div>
    </aside>
  </section>
</div>
<script>
const JOINTS = %JOINTS%;
const posTable = document.getElementById('pos-table');
const effTable = document.getElementById('eff-table');
const statusPill = document.getElementById('status-pill');
const footerStatus = document.getElementById('footer-status');
const camStatus = document.getElementById('cam-status');
const updatedAt = document.getElementById('updated-at');
const streamState = document.getElementById('stream-state');
const streamMeta = document.getElementById('stream-meta');
const spiRate = document.getElementById('spi-rate');
const spiRateMeta = document.getElementById('spi-rate-meta');
const spiTotal = document.getElementById('spi-total');
const spiLast = document.getElementById('spi-last');
const spiLastMeta = document.getElementById('spi-last-meta');

JOINTS.forEach(name => {
  let row = posTable.insertRow();
  row.innerHTML = `<td class="label">${name}</td><td id="pos-${name}">—</td><td id="deg-${name}">—</td>`;
  row = effTable.insertRow();
  row.innerHTML = `<td class="label">${name}</td><td id="eff-${name}">—</td>`;
});

const cam = document.getElementById('cam');
let prevUrl = null;
let lastFrameAt = 0;
let lastEvent = 0;

function formatAge(seconds) {
  if (seconds == null) return 'Never';
  if (seconds < 1) return '<1s';
  if (seconds < 60) return `${seconds.toFixed(1)}s`;
  const whole = Math.round(seconds);
  const mins = Math.floor(whole / 60);
  const rem = whole % 60;
  if (mins < 60) return `${mins}m ${rem}s`;
  const hrs = Math.floor(mins / 60);
  return `${hrs}h ${mins % 60}m`;
}

function setConnectionState(label, className, footer) {
  statusPill.textContent = label;
  statusPill.className = `status-pill ${className}`;
  footerStatus.textContent = footer;
}

function refreshDerivedState() {
  const now = Date.now();
  const frameAgeMs = lastFrameAt ? now - lastFrameAt : null;

  if (frameAgeMs != null && frameAgeMs < 2000) {
    camStatus.textContent = `Camera live · ${Math.round(frameAgeMs / 1000)}s frame age`;
  } else if (lastFrameAt) {
    camStatus.textContent = `Camera stale · ${Math.round(frameAgeMs / 1000)}s frame age`;
  } else {
    camStatus.textContent = 'Camera feed pending';
  }

  if (now - lastEvent > 2000) {
    setConnectionState('Stream stale', 'stale', 'No fresh telemetry in the last 2s');
    streamState.textContent = 'Stale';
    streamMeta.textContent = 'Joint telemetry has paused';
  }
}

async function streamVideo() {
  while (true) {
    try {
      const response = await fetch('/snapshot');
      if (!response.ok) {
        await new Promise(resolve => setTimeout(resolve, 200));
        continue;
      }
      const blob = await response.blob();
      const url = URL.createObjectURL(blob);
      if (prevUrl) URL.revokeObjectURL(prevUrl);
      prevUrl = url;
      await new Promise((resolve, reject) => {
        cam.onload = resolve;
        cam.onerror = reject;
        cam.src = url;
      });
      lastFrameAt = Date.now();
    } catch (_error) {
      await new Promise(resolve => setTimeout(resolve, 1000));
    }
  }
}
streamVideo();

const es = new EventSource('/events');
es.onmessage = event => {
  const data = JSON.parse(event.data);
  lastEvent = Date.now();

  setConnectionState('Telemetry live', 'online', 'Receiving joint and camera health updates');
  streamState.textContent = 'Live';
  streamMeta.textContent = 'ROS event stream connected';
  updatedAt.textContent = `Updated ${new Date().toLocaleTimeString()}`;

  const joints = data.joints;
  if (joints) {
    joints.name.forEach((name, index) => {
      const position = joints.position[index];
      const effort = joints.effort[index];
      document.getElementById(`pos-${name}`).textContent = position.toFixed(4);
      document.getElementById(`deg-${name}`).textContent = (position * 180 / Math.PI).toFixed(1);
      document.getElementById(`eff-${name}`).textContent = effort.toFixed(1);
    });
  }

  const spi = data.spi_health || {};
  const recent = spi.recoveries_last_minute ?? 0;
  const total = spi.recoveries_total ?? 0;
  const ageSec = spi.last_recovery_age_sec ?? null;
  spiRate.textContent = String(recent);
  spiTotal.textContent = String(total);
  spiLast.textContent = formatAge(ageSec);

  if (recent === 0) {
    spiRateMeta.textContent = 'No link resets in the last minute';
  } else if (recent === 1) {
    spiRateMeta.textContent = 'One SPI recovery in the last minute';
  } else {
    spiRateMeta.textContent = `${recent} SPI recoveries in the last minute`;
  }

  if (ageSec == null) {
    spiLastMeta.textContent = 'SPI link has not reset yet';
  } else {
    spiLastMeta.textContent = `Last reset was ${formatAge(ageSec)} ago`;
  }

  refreshDerivedState();
};
es.onerror = () => {
  setConnectionState('Disconnected', 'stale', 'Event stream disconnected');
  streamState.textContent = 'Offline';
  streamMeta.textContent = 'Waiting for the SSE connection to recover';
};

setInterval(refreshDerivedState, 1000);
</script>
</body>
</html>
""".replace("%JOINTS%", json.dumps(JOINT_NAMES))


class MonitorNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("monitor")
        self._loop = loop
        self.state_event = asyncio.Event()
        self.last_jpeg: bytes | None = None
        self.last_joints: dict | None = None
        self.last_spi_health = {
            "recoveries_total": 0,
            "recoveries_last_minute": 0,
            "last_recovery_age_sec": None,
        }

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
        self.create_subscription(
            String, "/camera/spi_health", self._on_spi_health, 10,
        )

    def _notify(self):
        self._loop.call_soon_threadsafe(self.state_event.set)

    def _state_payload(self) -> dict:
        return {
            "joints": self.last_joints,
            "spi_health": self.last_spi_health,
        }

    def _on_image(self, msg: CompressedImage):
        self.last_jpeg = bytes(msg.data)

    def _on_joints(self, msg: JointState):
        self.last_joints = {
            "name": list(msg.name),
            "position": list(msg.position),
            "effort": list(msg.effort),
        }
        self._notify()

    def _on_spi_health(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_spi_health = {
            "recoveries_total": int(payload.get("recoveries_total", 0)),
            "recoveries_last_minute": int(payload.get("recoveries_last_minute", 0)),
            "last_recovery_age_sec": payload.get("last_recovery_age_sec"),
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

    web.run_app(
        app,
        host="0.0.0.0",
        port=args.port,
        print=None,
        shutdown_timeout=0.5,
        handle_signals=False,
    )
