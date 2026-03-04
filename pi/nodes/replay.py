"""Replay a recorded episode from a LeRobot dataset.

Usage:
    uv run replay --repo-id my-org/demos --episode 0

Pulls the latest dataset revision, then reads actions via LeRobotDataset so the
replay path matches the sharded LeRobot v3 on-disk layout.
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from nodes._util import (
    ArmCommander,
    dataset_root,
    pull_dataset,
    shutdown_background_node,
    spin_in_background,
    wait_for_condition,
    wait_for_future,
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
        self.enter_replay_cli = self.create_client(Trigger, "/control/enter_replay_mode")
        self.exit_replay_cli = self.create_client(Trigger, "/control/exit_replay_mode")
        self._last_control = None
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.control_sub = self.create_subscription(
            String,
            "/control/status",
            self._on_control,
            control_qos,
        )

    def _on_control(self, msg: String):
        try:
            self._last_control = json.loads(msg.data)
        except json.JSONDecodeError:
            return


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
    spin_thread = spin_in_background(node)
    replay_mode_claimed = False

    def _set_replay_mode(active: bool):
        client = node.enter_replay_cli if active else node.exit_replay_cli
        action = "enter" if active else "exit"
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Control manager is unavailable")
        result = wait_for_future(client.call_async(Trigger.Request()), 2.0)
        if not result.success:
            raise RuntimeError(result.message or f"Failed to {action} replay mode")

    try:
        print("Connecting...", end="", flush=True)
        wait_for_condition(
            lambda: node.arm_pub.get_subscription_count() > 0,
            5.0,
            "arm subscriber",
        )
        print(" ready.")

        _set_replay_mode(True)
        replay_mode_claimed = True
        wait_for_condition(
            lambda: (
                node._last_control is not None
                and node._last_control.get("mode") == "replay"
                and node._last_control.get("outputs_converged")
            ),
            2.0,
            "replay mode to settle",
        )
        print("Replay mode active.")
    except Exception:
        if replay_mode_claimed:
            try:
                _set_replay_mode(False)
            except Exception:
                pass
        shutdown_background_node(node, spin_thread)
        raise

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
    finally:
        if replay_mode_claimed:
            try:
                _set_replay_mode(False)
            except Exception as exc:
                print(f"\nFailed to return to idle mode: {exc}")

    shutdown_background_node(node, spin_thread)


if __name__ == "__main__":
    main()
