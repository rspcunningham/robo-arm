"""Shared recording helpers for CLI and session-based recording."""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any, Callable

import numpy as np
from sensor_msgs.msg import Image as RosImage, JointState

from nodes._image import ros_image_to_rgb_pil
from nodes._util import JOINT_NAMES, dataset_root, pull_dataset

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
IMAGE_TOPIC = "/cam0/image_raw"


def _emit(log: Callable[[str], None] | None, message: str):
    if log is not None:
        log(message)


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


def open_or_create_dataset(repo_id: str, root: Path, fps: int, log: Callable[[str], None] | None = None) -> Any:
    LeRobotDataset = get_lerobot_dataset_cls()

    try:
        _emit(log, f"Pulling latest dataset from HF: {repo_id}...")
        pull_dataset(root, repo_id)
        _emit(log, f"Pulled latest dataset from HF: {repo_id}")
    except Exception as exc:
        _emit(log, f"Skipping dataset pull for {repo_id}: {exc}")

    discard_incomplete_dataset_root(root)
    if has_complete_dataset_root(root):
        try:
            dataset = LeRobotDataset(repo_id, root=root, download_videos=False)
            _emit(log, f"Opened existing dataset: {repo_id} ({dataset.num_episodes} episodes)")
            return dataset
        except Exception as exc:
            _emit(log, f"Recreating local dataset cache: {exc}")
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
    _emit(log, f"Created new dataset: {repo_id}")
    return dataset


def build_frame(task: str, joints: JointState, image_msg: RosImage) -> dict[str, Any]:
    rgb_image = ros_image_to_rgb_pil(image_msg)
    state = np.asarray(joints.position[:4], dtype=np.float32)
    return {
        "task": task,
        "observation.images.front": rgb_image,
        "observation.state": state,
        "action": state.copy(),
    }


class RecordingDataset:
    """Wrap a LeRobot dataset for repeated episode capture."""

    def __init__(
        self,
        repo_id: str,
        fps: int,
        public: bool = False,
        log: Callable[[str], None] | None = None,
    ):
        self.repo_id = repo_id
        self.public = public
        self.root = dataset_root(repo_id)
        self._log = log
        self.dataset = open_or_create_dataset(repo_id, self.root, fps, log=log)
        self.fps = int(self.dataset.fps)

    @property
    def num_episodes(self) -> int:
        return int(self.dataset.num_episodes)

    @property
    def num_frames(self) -> int:
        return int(self.dataset.num_frames)

    def add_frame(self, task: str, joints: JointState, image_msg: RosImage) -> None:
        self.dataset.add_frame(build_frame(task, joints, image_msg))

    def clear_episode_buffer(self) -> None:
        if self.dataset.episode_buffer is not None and self.dataset.episode_buffer["size"] > 0:
            self.dataset.clear_episode_buffer(delete_images=True)

    def save_episode(self) -> int:
        self.dataset.save_episode()
        return self.num_episodes - 1

    def finalize(self, push_to_hub: bool = True) -> None:
        self.dataset.finalize()
        if push_to_hub:
            self.dataset.push_to_hub(private=not self.public)
