"""Replay a recorded episode from a LeRobot-compatible dataset.

Usage:
    uv run replay --repo-id my-org/demos --episode 0

Pulls the latest dataset revision, then reads directly from parquet.
Uses daemon-thread spin so launch.py does NOT auto-detect this as a node.
"""

import argparse
import json
import time

import pyarrow.parquet as pq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from nodes._util import (
    ArmCommander,
    dataset_root,
    pull_dataset,
    spin_in_background,
    shutdown_and_exit,
)


class ReplayNode(ArmCommander, Node):
    def __init__(self):
        super().__init__("replay")
        self.arm_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.estop_cli = self.create_client(Trigger, "/emergency_stop")


def episode_data_path(root, episode_index: int, chunk_size: int):
    chunk_idx = episode_index // chunk_size
    file_idx = episode_index % chunk_size
    return root / "data" / f"chunk-{chunk_idx:03d}" / f"file-{file_idx:03d}.parquet"


def main():
    parser = argparse.ArgumentParser(description="Replay a recorded episode")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, e.g. my-org/demos")
    parser.add_argument("--episode", type=int, default=0, help="Episode index to replay")
    args = parser.parse_args()

    root = dataset_root(args.repo_id)
    try:
        print(f"Pulling latest dataset from HF: {args.repo_id}...", end="", flush=True)
        pull_dataset(root, args.repo_id)
        print(" ok.")
    except Exception as exc:
        print(f" skipped ({exc})")
    if not (root / "meta" / "info.json").exists():
        print(f"Dataset metadata not found in {root}")
        return
    with open(root / "meta" / "info.json") as f:
        info = json.load(f)
    fps = info["fps"]
    chunk_size = int(info.get("chunks_size", 1000))

    # New layout: one parquet file per episode. Fall back to the legacy monolithic file.
    data_path = episode_data_path(root, args.episode, chunk_size)
    if data_path.exists():
        table = pq.read_table(data_path)
        actions = table.column("action").to_pylist()
    else:
        legacy_data_path = root / "data" / "chunk-000" / "file-000.parquet"
        if not legacy_data_path.exists():
            print(f"Episode {args.episode} not found in {root}")
            return
        table = pq.read_table(legacy_data_path)
        ep_col = table.column("episode_index").to_pylist()
        action_col = table.column("action").to_pylist()
        actions = [action_col[i] for i, ep in enumerate(ep_col) if ep == args.episode]
        if not actions:
            print(f"Episode {args.episode} not found in {root}")
            return

    rclpy.init()
    node = ReplayNode()
    spin_in_background(node)

    print("Connecting...", end="", flush=True)
    while node.arm_pub.get_subscription_count() == 0:
        time.sleep(0.1)
    print(" ready.")

    interval = 1.0 / fps
    total = len(actions)
    print(f"Replaying episode {args.episode} ({total} frames at {fps} fps)")

    try:
        for i, action in enumerate(actions):
            t0 = time.time()
            node.publish_move(list(action))
            print(f"\r  frame {i + 1}/{total}", end="", flush=True)
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)
        print("\nDone")
    except KeyboardInterrupt:
        node.emergency_stop()
        print("\nAborted — emergency stop sent")

    shutdown_and_exit(node)


if __name__ == "__main__":
    main()
