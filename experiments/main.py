# pyright: reportMissingTypeStubs=false

from collections.abc import Mapping, Sequence
from typing import Protocol, cast, runtime_checkable

from lerobot.datasets.lerobot_dataset import LeRobotDataset

REPO_ID = "rspcunningham/test"


@runtime_checkable
class HasShape(Protocol):
    @property
    def shape(self) -> Sequence[int]: ...


@runtime_checkable
class HasDType(Protocol):
    @property
    def dtype(self) -> object: ...


@runtime_checkable
class FlatIndexable(Protocol):
    def __len__(self) -> int: ...

    def __getitem__(self, index: int, /) -> object: ...


@runtime_checkable
class Reshapable(Protocol):
    def reshape(self, *shape: int) -> FlatIndexable: ...


def _shape_of(value: object) -> tuple[int, ...] | None:
    if value is None:
        return None

    if isinstance(value, HasShape):
        return tuple(value.shape)

    return None


def _count_value(value: object) -> object:
    if value is None:
        return None

    if isinstance(value, Reshapable):
        flattened = value.reshape(-1)
        if len(flattened):
            return flattened[0]

    return value



def print_feature_summary(dataset: LeRobotDataset) -> None:
    print("features:")
    features = cast(Mapping[str, Mapping[str, object]], dataset.meta.features)
    for name, spec in features.items():
        shape = spec.get("shape")
        dtype = spec.get("dtype")
        print(f"  {name}: dtype={dtype}, shape={shape}")


def print_stats_summary(dataset: LeRobotDataset) -> None:
    print("stats:")
    stats = cast(Mapping[str, Mapping[str, object]] | None, dataset.meta.stats)
    if stats is None:
        print("  <none>")
        return

    for name, stat in stats.items():
        count_value = _count_value(stat.get("count"))
        mean = stat.get("mean")
        mean_shape = _shape_of(mean)
        print(f"  {name}: count={count_value}, mean_shape={mean_shape}")


def main() -> None:
    dataset = LeRobotDataset(REPO_ID)

    print(f"repo_id={dataset.repo_id}")
    print(f"episodes={dataset.num_episodes}")
    print(f"frames={len(dataset)}")
    print(f"fps={dataset.fps}")
    print(f"camera_keys={dataset.meta.camera_keys}")
    print(f"tasks={list(dataset.meta.tasks.index)}")
    print_feature_summary(dataset)
    print_stats_summary(dataset)

    sample = cast(Mapping[str, object], dataset[0])

    print(f"sample_keys={list(sample.keys())}")

    for key in dataset.meta.camera_keys:
        frame = sample[key]
        frame_shape = _shape_of(frame)
        frame_dtype = frame.dtype if isinstance(frame, HasDType) else None
        print(f"{key}: shape={frame_shape} dtype={frame_dtype}")


if __name__ == "__main__":
    main()
