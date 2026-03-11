"""Session-based recording manager for teleoperated dataset collection."""

from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage, JointState
from std_msgs.msg import String

from nodes._recording import IMAGE_TOPIC, RecordingDataset
from nodes._util import publish_or_ignore_shutdown, run_node

STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
WARNING_INTERVAL_SEC = 5.0


class RecordManagerNode(Node):
    def __init__(self):
        super().__init__("record_manager")

        self.status_pub = self.create_publisher(String, "/record/status", STATUS_QOS)
        self.create_subscription(String, "/record/command", self._on_command, COMMAND_QOS)
        self.create_subscription(RosImage, IMAGE_TOPIC, self._on_image, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(String, "/control/status", self._on_control, CONTROL_QOS)

        self._last_joints: JointState | None = None
        self._last_frame: RosImage | None = None
        self._control_mode = "idle"
        self._control_outputs_converged = False
        self._last_warning_monotonic = 0.0
        self._state_lock = threading.Lock()

        self._state = "idle"
        self._session_active = False
        self._episode_active = False
        self._task = ""
        self._repo_id = ""
        self._public = False
        self._dataset: RecordingDataset | None = None
        self._episodes_saved = 0
        self._current_episode_frames = 0
        self._last_error: str | None = None
        self._record_timer = None
        self._worker_thread: threading.Thread | None = None
        self._busy = False
        self._busy_action: str | None = None
        self._last_status_json: str | None = None

        self.create_timer(1.0, self._publish_status)
        self._publish_status()
        self.get_logger().info("Record manager ready")

    def _log(self, message: str):
        self.get_logger().info(message)

    def _set_error(self, message: str):
        with self._state_lock:
            self._last_error = message
        self.get_logger().warning(message)
        self._publish_status()

    def _clear_error(self):
        with self._state_lock:
            self._last_error = None

    def _teleop_ready(self) -> bool:
        with self._state_lock:
            return self._control_mode == "teleop" and self._control_outputs_converged

    def _status_message(self, snapshot: dict) -> str:
        state = snapshot["state"]
        if state == "opening":
            return "Opening dataset session"
        if state == "ready":
            return "Ready to start a new episode"
        if state == "recording":
            return "Recording episode"
        if state == "saving":
            return "Saving episode"
        if state == "finalizing":
            return "Finalizing dataset and uploading"
        return "Idle"

    def _status_snapshot(self) -> dict:
        with self._state_lock:
            dataset = self._dataset
            snapshot = {
                "state": self._state,
                "busy": self._busy,
                "busy_action": self._busy_action,
                "session_active": self._session_active,
                "episode_active": self._episode_active,
                "task": self._task,
                "repo_id": self._repo_id,
                "fps": dataset.fps if dataset is not None else None,
                "public": self._public,
                "episodes_saved": self._episodes_saved,
                "current_episode_frames": self._current_episode_frames,
                "dataset_num_episodes": dataset.num_episodes if dataset is not None else 0,
                "dataset_num_frames": dataset.num_frames if dataset is not None else 0,
                "control_mode": self._control_mode,
                "teleop_ready": self._control_mode == "teleop" and self._control_outputs_converged,
                "last_error": self._last_error,
            }
        snapshot["status_message"] = self._status_message(snapshot)
        return snapshot

    def _publish_status(self):
        payload = json.dumps(self._status_snapshot())
        if payload == self._last_status_json:
            return
        self._last_status_json = payload
        msg = String()
        msg.data = payload
        publish_or_ignore_shutdown(self.status_pub, msg)

    def _on_joints(self, msg: JointState):
        with self._state_lock:
            self._last_joints = msg

    def _on_image(self, msg: RosImage):
        with self._state_lock:
            self._last_frame = msg

    def _on_control(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._state_lock:
            self._control_mode = str(payload.get("mode", "idle"))
            self._control_outputs_converged = bool(payload.get("outputs_converged", False))
        self._publish_status()

    def _cancel_record_timer(self):
        if self._record_timer is not None:
            self._record_timer.cancel()
            self.destroy_timer(self._record_timer)
            self._record_timer = None

    def _start_worker(self, action: str, target):
        with self._state_lock:
            if self._busy:
                return False
            self._busy = True
            self._busy_action = action
            self._worker_thread = threading.Thread(target=target, daemon=True)
            self._worker_thread.start()
            return True

    def _begin_session(self, payload: dict):
        with self._state_lock:
            session_active = self._session_active
            last_joints = self._last_joints
            last_frame = self._last_frame
        if session_active:
            self._set_error("Recording session is already active")
            return
        with self._state_lock:
            busy = self._busy
        if busy:
            self._set_error("A recording operation is already in progress")
            return

        repo_id = str(payload.get("repo_id", "")).strip()
        task = str(payload.get("task", "")).strip()
        try:
            fps = int(payload.get("fps", 10))
        except (TypeError, ValueError):
            self._set_error("FPS must be a valid integer")
            return
        public = bool(payload.get("public", False))

        if not repo_id:
            self._set_error("Repository id is required")
            return
        if not task:
            self._set_error("Task is required")
            return
        if fps <= 0:
            self._set_error("FPS must be positive")
            return
        if last_joints is None or last_frame is None:
            self._set_error("Arm telemetry and camera frames must be available before recording")
            return

        with self._state_lock:
            self._state = "opening"
            self._task = task
            self._repo_id = repo_id
            self._public = public
            self._episodes_saved = 0
            self._current_episode_frames = 0
            self._last_error = None
        def _worker():
            try:
                dataset = RecordingDataset(repo_id, fps, public=public, log=self._log)
            except Exception as exc:
                with self._state_lock:
                    self._dataset = None
                    self._task = ""
                    self._repo_id = ""
                    self._public = False
                    self._state = "idle"
                    self._busy = False
                    self._last_error = str(exc)
                    self._busy_action = None
                self.get_logger().warning(str(exc))
                self._publish_status()
                return

            with self._state_lock:
                self._dataset = dataset
                self._session_active = True
                self._state = "ready"
                self._busy = False
                self._busy_action = None
            self._publish_status()

        self._start_worker("open_session", _worker)
        self._publish_status()

    def _start_episode(self):
        with self._state_lock:
            session_active = self._session_active
            dataset = self._dataset
            episode_active = self._episode_active
            teleop_ready = self._control_mode == "teleop" and self._control_outputs_converged
            last_joints = self._last_joints
            last_frame = self._last_frame
        if not session_active or dataset is None:
            self._set_error("Begin a recording session first")
            return
        with self._state_lock:
            busy = self._busy
        if busy:
            self._set_error("A recording operation is already in progress")
            return
        if episode_active:
            self._set_error("An episode is already recording")
            return
        if not teleop_ready:
            self._set_error("Teleop must be enabled before starting an episode")
            return
        if last_joints is None or last_frame is None:
            self._set_error("Arm telemetry and camera frames must be available before recording")
            return

        with self._state_lock:
            self._last_error = None
            self._state = "recording"
            self._episode_active = True
            self._current_episode_frames = 0
        self._record_timer = self.create_timer(1.0 / dataset.fps, self._record_tick)
        self._publish_status()

    def _finish_episode(self):
        with self._state_lock:
            episode_active = self._episode_active
            dataset = self._dataset
            current_episode_frames = self._current_episode_frames
        if not episode_active or dataset is None:
            self._set_error("No episode is currently recording")
            return
        with self._state_lock:
            busy = self._busy
        if busy:
            self._set_error("A recording operation is already in progress")
            return

        self._cancel_record_timer()
        with self._state_lock:
            self._episode_active = False

        if current_episode_frames <= 0:
            dataset.clear_episode_buffer()
            with self._state_lock:
                self._state = "ready"
                self._last_error = "No frames were captured for this episode"
                self._current_episode_frames = 0
            self.get_logger().warning("No frames were captured for this episode")
            self._publish_status()
            return

        with self._state_lock:
            self._state = "saving"
            self._last_error = None
        def _worker():
            try:
                dataset.save_episode()
            except Exception as exc:
                dataset.clear_episode_buffer()
                with self._state_lock:
                    self._state = "ready"
                    self._current_episode_frames = 0
                    self._busy = False
                    self._last_error = f"Failed to save episode: {exc}"
                    self._busy_action = None
                self.get_logger().warning(f"Failed to save episode: {exc}")
                self._publish_status()
                return

            with self._state_lock:
                self._episodes_saved += 1
                self._current_episode_frames = 0
                self._state = "ready"
                self._busy = False
                self._busy_action = None
            self._publish_status()

        self._start_worker("save_episode", _worker)
        self._publish_status()

    def _finish_session(self):
        with self._state_lock:
            session_active = self._session_active
            dataset = self._dataset
            episode_active = self._episode_active
            episodes_saved = self._episodes_saved
        if not session_active or dataset is None:
            self._set_error("No recording session is active")
            return
        with self._state_lock:
            busy = self._busy
        if busy:
            self._set_error("A recording operation is already in progress")
            return
        if episode_active:
            self._set_error("Finish the current episode before leaving the session")
            return

        with self._state_lock:
            self._state = "finalizing"
            self._last_error = None
        def _worker():
            last_error = None
            try:
                if episodes_saved > 0:
                    dataset.finalize(push_to_hub=True)
            except Exception as exc:
                last_error = f"Failed to finalize dataset: {exc}"
                self.get_logger().warning(last_error)
            finally:
                with self._state_lock:
                    self._dataset = None
                    self._session_active = False
                    self._episode_active = False
                    self._state = "idle"
                    self._task = ""
                    self._repo_id = ""
                    self._public = False
                    self._current_episode_frames = 0
                    self._busy = False
                    self._last_error = last_error
                    self._busy_action = None
                self._publish_status()

        self._start_worker("finalize_session", _worker)
        self._publish_status()

    def _record_tick(self):
        with self._state_lock:
            episode_active = self._episode_active
            dataset = self._dataset
            joints = self._last_joints
            frame = self._last_frame
            task = self._task
        if not episode_active or dataset is None:
            return

        if joints is None or frame is None or len(joints.position) < 4:
            return

        try:
            dataset.add_frame(task, joints, frame)
        except ValueError as exc:
            now = time.monotonic()
            if now - self._last_warning_monotonic >= WARNING_INTERVAL_SEC:
                self._last_warning_monotonic = now
                self.get_logger().warning(f"Dropping unsupported camera frame: {exc}")
            return
        except Exception as exc:
            self._cancel_record_timer()
            with self._state_lock:
                self._episode_active = False
                self._state = "ready"
                self._last_error = f"Recording aborted: {exc}"
            self.get_logger().warning(f"Recording aborted: {exc}")
            self._publish_status()
            return

        with self._state_lock:
            self._current_episode_frames += 1
            frame_count = self._current_episode_frames
        if frame_count % 10 == 0:
            self._publish_status()

    def _on_command(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self._set_error("Received invalid record command JSON")
            return

        action = str(payload.get("action", "")).strip()
        if not action:
            self._set_error("Record command is missing an action")
            return

        if action == "begin_session":
            self._begin_session(payload)
            return
        if action == "start":
            self._start_episode()
            return
        if action == "done":
            self._finish_episode()
            return
        if action == "finish":
            self._finish_session()
            return

        self._set_error(f"Unknown record action: {action}")

    def destroy_node(self):
        self._cancel_record_timer()
        super().destroy_node()


def main():
    rclpy.init()
    run_node(RecordManagerNode())
