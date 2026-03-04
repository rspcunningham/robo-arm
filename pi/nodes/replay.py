"""Replay a recorded episode from a LeRobot dataset.

Usage:
    uv run replay --repo-id my-org/demos --episode 0

Pulls the latest dataset revision, then reads actions via LeRobotDataset so the
replay path matches the sharded LeRobot v3 on-disk layout.
"""

import argparse
import time

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


def get_lerobot_dataset_cls():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError(
            "The replay command now uses LeRobotDataset. Install `lerobot>=0.4.4,<0.5` first."
        ) from exc
    return LeRobotDataset


class ReplayNode(ArmCommander, Node):
    def __init__(self):
        super().__init__("replay")
        self.arm_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.estop_cli = self.create_client(Trigger, "/emergency_stop")


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

    try:
        LeRobotDataset = get_lerobot_dataset_cls()
        dataset = LeRobotDataset(
            args.repo_id,
            root=root,
            episodes=[args.episode],
            download_videos=False,
        )
    except Exception as exc:
        print(f"Failed to open episode {args.episode}: {exc}")
        return

    actions = [dataset[i]["action"] for i in range(len(dataset))]
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

    interval = 1.0 / dataset.fps
    total = len(actions)
    print(f"Replaying episode {args.episode} ({total} frames at {dataset.fps} fps)")

    try:
        for i, action in enumerate(actions):
            t0 = time.time()
            move = action.tolist() if hasattr(action, "tolist") else list(action)
            node.publish_move(move)
            print(f"\r  frame {i + 1}/{total}", end="", flush=True)
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)
        print("\nDone")
    except KeyboardInterrupt:
        node.emergency_stop()
        print("\nAborted - emergency stop sent")

    shutdown_and_exit(node)


if __name__ == "__main__":
    main()
