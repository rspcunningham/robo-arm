"""Record teleop episodes to a LeRobot-compatible dataset.

Usage:
    uv run record --repo-id my-org/demos --fps 10 --task "pick up block" --num-episodes 3

Creates a LeRobot v3.0 dataset with:
  - observation.images.front  (480, 640, 3) image
  - observation.state         (4,) float32 joint positions
  - action                    (4,) float32 (= current state during teleop)

Writes directly with HF datasets + PyArrow, then pushes to a HF dataset repo.
Uses daemon-thread spin so launch.py does NOT auto-detect this as a node.
"""

import argparse
import io
import json
import threading
import time
from pathlib import Path

import datasets
import numpy as np
import pandas as pd
import pyarrow.parquet as pq
import rclpy
from datasets.table import embed_table_storage
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import SetBool

from nodes._util import (
    JOINT_NAMES,
    dataset_root,
    pull_dataset,
    push_dataset,
    spin_in_background,
    shutdown_and_exit,
)

CODEBASE_VERSION = "v3.0"
LEROBOT_FEATURES = {
    "observation.images.front": {
        "dtype": "image",
        "shape": [480, 640, 3],
        "names": ["height", "width", "channels"],
    },
    "observation.state": {
        "dtype": "float32",
        "shape": [4],
        "names": JOINT_NAMES,
    },
    "action": {
        "dtype": "float32",
        "shape": [4],
        "names": JOINT_NAMES,
    },
}


def get_hf_features():
    """Build HF datasets.Features matching LeRobot v3.0 schema."""
    return datasets.Features({
        "timestamp": datasets.Value("float32"),
        "frame_index": datasets.Value("int64"),
        "episode_index": datasets.Value("int64"),
        "index": datasets.Value("int64"),
        "task_index": datasets.Value("int64"),
        "observation.images.front": datasets.Image(),
        "observation.state": datasets.Sequence(
            length=4, feature=datasets.Value("float32"),
        ),
        "action": datasets.Sequence(
            length=4, feature=datasets.Value("float32"),
        ),
    })


def create_dataset(root: Path, fps: int) -> dict:
    """Create a new empty dataset directory and return its info dict."""
    all_features = {
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
        **LEROBOT_FEATURES,
    }

    info = {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": "so100",
        "total_episodes": 0,
        "total_frames": 0,
        "total_tasks": 0,
        "chunks_size": 1000,
        "data_files_size_in_mb": 100,
        "video_files_size_in_mb": 200,
        "fps": fps,
        "splits": {"train": "0:0"},
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
        "features": all_features,
    }

    (root / "meta").mkdir(parents=True, exist_ok=True)
    (root / "meta" / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (root / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)

    with open(root / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    # Empty tasks DataFrame
    pd.DataFrame({"task_index": pd.Series([], dtype="int64")}).to_parquet(
        root / "meta" / "tasks.parquet",
    )

    return info


def load_dataset_info(root: Path) -> dict:
    """Load existing dataset info.json."""
    with open(root / "meta" / "info.json") as f:
        return json.load(f)


def episode_paths(root: Path, info: dict, ep_idx: int) -> tuple[Path, Path, int, int]:
    """Return data/meta parquet paths for an episode."""
    chunk_size = int(info["chunks_size"])
    chunk_idx = ep_idx // chunk_size
    file_idx = ep_idx % chunk_size

    data_path = root / "data" / f"chunk-{chunk_idx:03d}" / f"file-{file_idx:03d}.parquet"
    meta_path = (
        root / "meta" / "episodes" / f"chunk-{chunk_idx:03d}" / f"file-{file_idx:03d}.parquet"
    )
    data_path.parent.mkdir(parents=True, exist_ok=True)
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    return data_path, meta_path, chunk_idx, file_idx


def save_episode(root: Path, info: dict, frames: list[dict], task: str):
    """Write one episode to the dataset (data parquet + episode metadata + tasks)."""
    ep_idx = info["total_episodes"]
    global_start = info["total_frames"]
    n = len(frames)
    fps = info["fps"]
    data_path, ep_path, chunk_idx, file_idx = episode_paths(root, info, ep_idx)

    # --- Resolve task index ---
    tasks_path = root / "meta" / "tasks.parquet"
    tasks_df = pd.read_parquet(tasks_path)
    if task in tasks_df.index:
        task_idx = int(tasks_df.loc[task, "task_index"])
    else:
        task_idx = len(tasks_df)
        tasks_df = pd.concat([
            tasks_df,
            pd.DataFrame({"task_index": [task_idx]}, index=[task]),
        ])
        tasks_df.to_parquet(tasks_path)
    info["total_tasks"] = len(tasks_df)

    # --- Build data columns ---
    images = [Image.open(io.BytesIO(bytes(f["jpeg"]))) for f in frames]
    states = [f["state"] for f in frames]

    ep_dict = {
        "timestamp": [i / fps for i in range(n)],
        "frame_index": list(range(n)),
        "episode_index": [ep_idx] * n,
        "index": list(range(global_start, global_start + n)),
        "task_index": [task_idx] * n,
        "observation.images.front": images,
        "observation.state": states,
        "action": states,  # during teleop, action = current state
    }

    # Convert to HF Dataset and embed images into parquet
    hf_features = get_hf_features()
    ep_dataset = datasets.Dataset.from_dict(ep_dict, features=hf_features, split="train")
    ep_dataset = ep_dataset.with_format("arrow")
    ep_dataset = ep_dataset.map(embed_table_storage, batched=False)
    table = ep_dataset.with_format("arrow")[:]

    # One parquet file per episode keeps HF sync incremental.
    pq.write_table(table, data_path)

    # --- Episode metadata ---
    states_np = np.array(states, dtype=np.float32)
    ep_stats = {}
    for name, arr in [("observation.state", states_np), ("action", states_np)]:
        ep_stats[f"stats/{name}/mean"] = [arr.mean(axis=0).tolist()]
        ep_stats[f"stats/{name}/std"] = [arr.std(axis=0).tolist()]
        ep_stats[f"stats/{name}/min"] = [arr.min(axis=0).tolist()]
        ep_stats[f"stats/{name}/max"] = [arr.max(axis=0).tolist()]

    ep_meta = {
        "episode_index": [ep_idx],
        "tasks": [[task]],
        "length": [n],
        "dataset_from_index": [global_start],
        "dataset_to_index": [global_start + n],
        "data/chunk_index": [chunk_idx],
        "data/file_index": [file_idx],
        "meta/episodes/chunk_index": [chunk_idx],
        "meta/episodes/file_index": [file_idx],
        **ep_stats,
    }

    import pyarrow as pa
    ep_table = pa.table({k: pa.array(v) for k, v in ep_meta.items()})
    pq.write_table(ep_table, ep_path)

    # --- Update info ---
    info["total_episodes"] = ep_idx + 1
    info["total_frames"] = global_start + n
    info["splits"]["train"] = f"0:{ep_idx + 1}"

    with open(root / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=2)


class RecordNode(Node):
    def __init__(self):
        super().__init__("record")
        self.torque_cli = self.create_client(SetBool, "/torque_enable")
        self._last_joints = None
        self._last_frame = None
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(
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
    spin_in_background(node)

    # Wait for arm + camera
    print("Waiting for arm...", end="", flush=True)
    while node._last_joints is None:
        time.sleep(0.1)
    print(" ok. Waiting for camera...", end="", flush=True)
    while node._last_frame is None:
        time.sleep(0.1)
    print(" ok.")

    # Release torque so the arm can be guided by hand
    if node.torque_cli.wait_for_service(timeout_sec=2.0):
        req = SetBool.Request()
        req.data = False
        node.torque_cli.call_async(req)
        print("Torque released.")
    else:
        print("Warning: torque service not available")

    # Create or load existing dataset
    root = dataset_root(args.repo_id)
    try:
        print(f"Pulling latest dataset from HF: {args.repo_id}...", end="", flush=True)
        pull_dataset(root, args.repo_id, create_if_missing=True, private=not args.public)
        print(" ok.")
    except Exception as exc:
        print(f" skipped ({exc})")
    if (root / "meta" / "info.json").exists():
        info = load_dataset_info(root)
        print(f"Opened existing dataset: {args.repo_id} ({info['total_episodes']} episodes)")
    else:
        info = create_dataset(root, args.fps)
        print(f"Created new dataset: {args.repo_id}")

    interval = 1.0 / args.fps
    stop_flag = threading.Event()

    def _wait_enter():
        """Block until Enter is pressed, then set stop_flag."""
        input()
        stop_flag.set()

    episode = 0
    try:
        while episode < args.num_episodes:
            input(f"\nPress Enter to start episode {info['total_episodes']} "
                  f"({episode + 1}/{args.num_episodes})...")
            frame_buf = []
            stop_flag.clear()
            enter_thread = threading.Thread(target=_wait_enter, daemon=True)
            enter_thread.start()
            print(f"Recording at {args.fps} fps. Press Enter to stop.")
            while not stop_flag.is_set():
                t0 = time.time()

                js = node._last_joints
                img_msg = node._last_frame
                if js is None or img_msg is None or len(js.position) < 4:
                    elapsed = time.time() - t0
                    if elapsed < interval:
                        time.sleep(interval - elapsed)
                    continue

                frame_buf.append({
                    "state": list(js.position[:4]),
                    "jpeg": bytes(img_msg.data),
                })
                print(f"\r  frame {len(frame_buf)}", end="", flush=True)

                elapsed = time.time() - t0
                if elapsed < interval:
                    time.sleep(interval - elapsed)

            if not frame_buf:
                print("\nNo frames recorded, skipping episode.")
                continue

            print(f"\nSaving episode ({len(frame_buf)} frames)...", end="", flush=True)
            save_episode(root, info, frame_buf, args.task)
            print(f" done (episode {info['total_episodes'] - 1})")
            try:
                print(f"Pushing dataset to HF: {args.repo_id}...", end="", flush=True)
                push_dataset(
                    root,
                    args.repo_id,
                    private=not args.public,
                    commit_message=f"Add episode {info['total_episodes'] - 1}",
                )
                print(" ok.")
            except Exception as exc:
                print(f" failed ({exc})")
            episode += 1

    except KeyboardInterrupt:
        print("\nFinished recording.")

    print(f"Dataset: {root} ({info['total_episodes']} episodes, {info['total_frames']} frames)")

    shutdown_and_exit(node)


if __name__ == "__main__":
    main()
