"""Shared helpers for robot nodes."""

import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

JOINT_NAMES = ["base", "shoulder", "elbow", "hand"]


def _is_shutdown_publish_error(exc: Exception) -> bool:
    message = str(exc).lower()
    return (not rclpy.ok()) or "context is invalid" in message


def publish_or_ignore_shutdown(publisher, msg) -> bool:
    """Publish unless ROS is shutting down; suppress invalid-context shutdown races."""
    try:
        publisher.publish(msg)
    except Exception as exc:
        if _is_shutdown_publish_error(exc):
            return False
        raise
    return True


def dataset_root(repo_id: str) -> Path:
    """Local working copy for a dataset repo id."""
    return Path.home() / "data" / "lerobot" / repo_id


def pull_dataset(root: Path, repo_id: str, create_if_missing: bool = False, private: bool = True):
    """Pull the latest dataset files from HF into a local working copy."""
    from huggingface_hub import create_repo, snapshot_download

    root.mkdir(parents=True, exist_ok=True)
    if create_if_missing:
        create_repo(repo_id=repo_id, repo_type="dataset", private=private, exist_ok=True)
    snapshot_download(
        repo_id=repo_id,
        repo_type="dataset",
        local_dir=root,
    )


def push_dataset(root: Path, repo_id: str, private: bool = True, commit_message: str | None = None):
    """Push the local dataset working copy to HF."""
    from huggingface_hub import create_repo, upload_folder

    create_repo(repo_id=repo_id, repo_type="dataset", private=private, exist_ok=True)
    upload_folder(
        repo_id=repo_id,
        repo_type="dataset",
        folder_path=root,
        commit_message=commit_message,
    )


def run_node(node: Node):
    """Spin forever, handle KeyboardInterrupt, cleanup. For background nodes."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exc:
        if not _is_shutdown_publish_error(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def spin_in_background(node: Node):
    """Background spin thread with ExternalShutdownException catch."""
    def _spin():
        try:
            rclpy.spin(node)
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception as exc:
            if not _is_shutdown_publish_error(exc):
                raise

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return thread


def wait_for_future(future, timeout_sec: float):
    """Wait for a ROS future while another thread spins the node."""
    deadline = time.monotonic() + timeout_sec
    while not future.done():
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for ROS future")
        time.sleep(0.01)
    return future.result()


def wait_for_condition(
    predicate,
    timeout_sec: float,
    description: str,
    poll_interval_sec: float = 0.05,
):
    """Wait for a condition to become true, then return."""
    deadline = time.monotonic() + timeout_sec
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Timed out waiting for {description}")
        time.sleep(poll_interval_sec)


def shutdown_background_node(node: Node, spin_thread: threading.Thread, join_timeout_sec: float = 3.0):
    """Shutdown ROS, wait for the spin thread to stop, then destroy the node."""
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    spin_thread.join(timeout=join_timeout_sec)
    if spin_thread.is_alive():
        raise RuntimeError("ROS spin thread did not exit cleanly")
    node.destroy_node()


class ArmCommander:
    """Mixin: publish_move(pos, spd=None) and emergency_stop().

    Expects self.arm_pub and self.estop_cli on the class.
    """

    def publish_move(self, pos: list[float], spd: float | None = None):
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = pos
        if spd is not None:
            msg.velocity = [spd]
        publish_or_ignore_shutdown(self.arm_pub, msg)

    def emergency_stop(self, timeout_sec: float = 1.0):
        if not self.estop_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Emergency stop service is unavailable")
        future = self.estop_cli.call_async(Trigger.Request())
        return wait_for_future(future, timeout_sec)
