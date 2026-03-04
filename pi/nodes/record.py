"""Record teleop episodes into a LeRobot v3 dataset.

Usage:
    uv run record --repo-id my-org/demos --fps 10 --task "pick up block" --num-episodes 3

Uses the official LeRobot writer API so the dataset stays compatible with the
LeRobot loaders, sharding rules, metadata, and Hub tooling.
"""

import argparse
import io
import shutil
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import SetBool

from nodes._util import (
    JOINT_NAMES,
    dataset_root,
    pull_dataset,
    shutdown_background_node,
    spin_in_background,
    wait_for_condition,
    wait_for_future,
)

LEROBOT_FEATURES = {
    "observation.images.front": {
        "dtype": "video",
        "shape": [480, 640, 3],
        "names": ["height", "width", "channels"],
    },
    "observation.state": {
        "dtype": "float32",
        "shape": (4,),
        "names": JOINT_NAMES,
    },
    "action": {
        "dtype": "float32",
        "shape": (4,),
        "names": JOINT_NAMES,
    },
}


def has_complete_dataset_root(root: Path) -> bool:
    required_files = (
        root / "meta" / "info.json",
        root / "meta" / "tasks.parquet",
    )
    return all(path.exists() for path in required_files)


def get_lerobot_dataset_cls():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError(
            "The record command now uses LeRobotDataset. Install `lerobot>=0.4.4,<0.5` first."
        ) from exc
    return LeRobotDataset


def discard_incomplete_dataset_root(root: Path) -> None:
    """Remove an empty or partial dataset root before creating a fresh dataset."""
    if not root.exists() or has_complete_dataset_root(root):
        return

    if any(root.iterdir()):
        shutil.rmtree(root)
    else:
        root.rmdir()


def open_or_create_dataset(repo_id: str, root: Path, fps: int) -> Any:
    LeRobotDataset = get_lerobot_dataset_cls()

    try:
        print(f"Pulling latest dataset from HF: {repo_id}...", end="", flush=True)
        pull_dataset(root, repo_id)
        print(" ok.")
    except Exception as exc:
        print(f" skipped ({exc})")

    discard_incomplete_dataset_root(root)
    if has_complete_dataset_root(root):
        try:
            dataset = LeRobotDataset(repo_id, root=root, download_videos=False)
            print(f"Opened existing dataset: {repo_id} ({dataset.num_episodes} episodes)")
            return dataset
        except Exception as exc:
            print(f"Recreating local dataset cache ({exc})")
            shutil.rmtree(root)

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=LEROBOT_FEATURES,
        root=root,
        robot_type="so100",
        use_videos=True,
        vcodec="h264",
    )
    print(f"Created new dataset: {repo_id}")
    return dataset


def build_frame(task: str, joints: JointState, image_msg: CompressedImage) -> dict[str, Any]:
    with Image.open(io.BytesIO(bytes(image_msg.data))) as image:
        rgb_image = image.convert("RGB")

    state = np.asarray(joints.position[:4], dtype=np.float32)
    return {
        "task": task,
        "observation.images.front": rgb_image,
        "observation.state": state,
        "action": state.copy(),
    }


class RecordNode(Node):
    def __init__(self):
        super().__init__("record")
        self.torque_cli = self.create_client(SetBool, "/torque_enable")
        self._last_joints = None
        self._last_frame = None
        self._joints_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joints,
            10,
        )
        self._image_sub = self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_joints(self, msg: JointState):
        self._last_joints = msg

    def _on_image(self, msg: CompressedImage):
        self._last_frame = msg


def main():
    parser = argparse.ArgumentParser(description="Record teleop episodes")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, e.g. my-org/demos")
    parser.add_argument("--fps", type=int, default=10, help="Recording FPS (default: 10)")
    parser.add_argument("--task", default="", help="Task description")
    parser.add_argument("--num-episodes", type=int, default=1, help="Number of episodes to record")
    parser.add_argument(
        "--public",
        action="store_true",
        help="Create the HF dataset repo as public instead of private",
    )
    args = parser.parse_args()

    rclpy.init()
    node = RecordNode()
    spin_thread = spin_in_background(node)

    try:
        print("Waiting for arm...", end="", flush=True)
        wait_for_condition(lambda: node._last_joints is not None, 5.0, "joint telemetry")
        print(" ok. Waiting for camera...", end="", flush=True)
        wait_for_condition(lambda: node._last_frame is not None, 5.0, "camera frames")
        print(" ok.")

        if not node.torque_cli.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Torque service is unavailable")
        req = SetBool.Request()
        req.data = False
        wait_for_future(node.torque_cli.call_async(req), 2.0)
        print("Torque released.")
    except Exception:
        shutdown_background_node(node, spin_thread)
        raise

    root = dataset_root(args.repo_id)
    dataset = open_or_create_dataset(args.repo_id, root, args.fps)
    if dataset.fps != args.fps:
        print(f"Using dataset fps {dataset.fps} (ignoring --fps {args.fps}).")

    interval = 1.0 / dataset.fps
    stop_flag = threading.Event()
    saved_episodes = 0

    def _wait_enter():
        """Block until Enter is pressed, then set stop_flag."""
        try:
            input()
        except EOFError:
            pass
        stop_flag.set()

    episode = 0
    try:
        while episode < args.num_episodes:
            try:
                input(
                    f"\nPress Enter to start episode {dataset.num_episodes} "
                    f"({episode + 1}/{args.num_episodes})..."
                )
            except EOFError:
                print("\nInput closed before recording started.")
                break
            try:
                input("Press Enter again to begin recording...")
            except EOFError:
                print("\nInput closed before recording started.")
                break
            frame_count = 0
            stop_flag.clear()
            enter_thread = threading.Thread(target=_wait_enter, daemon=True)
            enter_thread.start()
            print(f"Recording at {dataset.fps} fps. Press Enter to stop.")
            while not stop_flag.is_set():
                t0 = time.time()

                js = node._last_joints
                img_msg = node._last_frame
                if js is None or img_msg is None or len(js.position) < 4:
                    elapsed = time.time() - t0
                    if elapsed < interval:
                        time.sleep(interval - elapsed)
                    continue

                dataset.add_frame(build_frame(args.task, js, img_msg))
                frame_count += 1
                print(f"\r  frame {frame_count}", end="", flush=True)

                elapsed = time.time() - t0
                if elapsed < interval:
                    time.sleep(interval - elapsed)

            if frame_count == 0:
                print("\nNo frames recorded, skipping episode.")
                continue

            print(f"\nSaving episode ({frame_count} frames)...", end="", flush=True)
            dataset.save_episode()
            saved_episodes += 1
            print(f" done (episode {dataset.num_episodes - 1})")
            episode += 1

    except KeyboardInterrupt:
        if dataset.episode_buffer is not None and dataset.episode_buffer["size"] > 0:
            dataset.clear_episode_buffer(delete_images=True)
            print("\nDiscarded the in-progress episode.")
        print("\nFinished recording.")
    finally:
        if saved_episodes:
            print("Finalizing dataset...", end="", flush=True)
            dataset.finalize()
            print(" ok.")
            try:
                print(f"Pushing dataset to HF: {args.repo_id}...", end="", flush=True)
                dataset.push_to_hub(private=not args.public)
                print(" ok.")
            except Exception as exc:
                print(f" failed ({exc})")

        print(f"Dataset: {root} ({dataset.num_episodes} episodes, {dataset.num_frames} frames)")
        shutdown_background_node(node, spin_thread)


if __name__ == "__main__":
    main()
