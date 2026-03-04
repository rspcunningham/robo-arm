import os
import sys
import warnings
from contextlib import contextmanager

warnings.filterwarnings("ignore", category=SyntaxWarning, module=r"multiprocess\.connection")

from lerobot.datasets.lerobot_dataset import LeRobotDataset


REPO_ID = "rspcunningham/test"


@contextmanager
def suppress_stderr():
    try:
        stderr_fd = sys.stderr.fileno()
    except (AttributeError, OSError):
        yield
        return

    saved_stderr = os.dup(stderr_fd)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull_fd, stderr_fd)
        yield
    finally:
        os.dup2(saved_stderr, stderr_fd)
        os.close(saved_stderr)
        os.close(devnull_fd)


def print_feature_summary(dataset: LeRobotDataset) -> None:
    print("features:")
    for name, spec in dataset.meta.features.items():
        shape = spec.get("shape")
        dtype = spec.get("dtype")
        print(f"  {name}: dtype={dtype}, shape={shape}")


def print_stats_summary(dataset: LeRobotDataset) -> None:
    print("stats:")
    for name, stat in dataset.meta.stats.items():
        count = stat.get("count")
        count_value = int(count.reshape(-1)[0]) if hasattr(count, "reshape") else count
        mean = stat.get("mean")
        mean_shape = tuple(mean.shape) if hasattr(mean, "shape") else None
        print(f"  {name}: count={count_value}, mean_shape={mean_shape}")


def main() -> None:
    dataset = LeRobotDataset(REPO_ID)

    print(f"repo_id={dataset.repo_id}")
    print(f"episodes={dataset.num_episodes}")
    print(f"frames={len(dataset)}")
    print(f"fps={dataset.fps}")
    print(f"camera_keys={dataset.meta.camera_keys}")
    if dataset.meta.tasks is not None:
        print(f"tasks={list(dataset.meta.tasks.index)}")
    print_feature_summary(dataset)
    print_stats_summary(dataset)

    with suppress_stderr():
        sample = dataset[0]

    print(f"sample_keys={list(sample.keys())}")

    for key in dataset.meta.camera_keys:
        frame = sample[key]
        print(f"{key}: shape={tuple(frame.shape)} dtype={frame.dtype}")


if __name__ == "__main__":
    main()
