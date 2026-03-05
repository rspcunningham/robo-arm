# %%
from __future__ import annotations

from collections.abc import Mapping, Sequence
from time import perf_counter_ns
from typing import Callable, cast

import torch

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.pi05 import PI05Policy

MODEL_ID = "lerobot/pi05_base"
DATASET_ID = "rspcunningham/test"
VIDEO_BACKEND = "pyav"
DEVICE = torch.device("mps")

# Dataset camera key -> PI0.5 camera key.
CAMERA_MAP_DATASET_TO_PI = {
    "observation.images.front": "observation.images.base_0_rgb",
}

# Robot joint index -> PI0.5 joint index.
ROBOT_TO_PI_JOINT_MAP = {
    0: 0,
    1: 1,
    2: 2,
    3: 6,
}

Sample = dict[str, object]
TensorOrSeq = torch.Tensor | Sequence[float]

# %%

def remap_vector_to_target_dim(
    source: torch.Tensor, target_dim: int, source_to_target_index_map: dict[int, int]
) -> torch.Tensor:
    remapped = torch.zeros(*source.shape[:-1], target_dim, dtype=source.dtype, device=source.device)
    for source_idx, target_idx in source_to_target_index_map.items():
        if source_idx < source.shape[-1] and target_idx < target_dim:
            remapped[..., target_idx] = source[..., source_idx]
    return remapped


def remap_pi_to_robot(pi_vector: torch.Tensor, robot_to_pi_index_map: dict[int, int]) -> torch.Tensor:
    robot_dim = max(robot_to_pi_index_map.keys()) + 1
    robot_vector = torch.zeros(*pi_vector.shape[:-1], robot_dim, dtype=pi_vector.dtype, device=pi_vector.device)
    for robot_idx, pi_idx in robot_to_pi_index_map.items():
        if pi_idx < pi_vector.shape[-1]:
            robot_vector[..., robot_idx] = pi_vector[..., pi_idx]
    return robot_vector


def get_pi_image_keys(policy: PI05Policy) -> list[str]:
    input_features = policy.config.input_features
    if input_features is None:
        return []
    return [key for key in input_features.keys() if key.startswith("observation.images.")]


def build_camera_mapping(
    expected_pi_image_keys: list[str], dataset_to_pi_camera_map: dict[str, str]
) -> dict[str, str]:
    resolved_mapping: dict[str, str] = {}
    for dataset_camera_key, pi_camera_key in dataset_to_pi_camera_map.items():
        if pi_camera_key in expected_pi_image_keys:
            resolved_mapping[pi_camera_key] = dataset_camera_key
    return resolved_mapping


def _to_1d_float_tensor(values: TensorOrSeq) -> torch.Tensor:
    if isinstance(values, torch.Tensor):
        tensor = values.to(dtype=torch.float32)
    else:
        tensor = torch.as_tensor(values, dtype=torch.float32)
    return tensor.reshape(-1)


def _sync_device(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device=device)
    elif device.type == "mps":
        torch.mps.synchronize()


class PIInferencePipeline:
    model_id: str
    device: torch.device
    robot_to_pi_joint_map: dict[int, int]
    policy: PI05Policy
    preprocess_fn: Callable[[Sample], dict[str, object]]
    postprocess_fn: Callable[[torch.Tensor], torch.Tensor]
    expected_pi_image_keys: list[str]
    camera_mapping: dict[str, str]

    def __init__(
        self,
        model_id: str = MODEL_ID,
        device: torch.device = DEVICE,
        dataset_to_pi_camera_map: dict[str, str] = CAMERA_MAP_DATASET_TO_PI,
        robot_to_pi_joint_map: dict[int, int] = ROBOT_TO_PI_JOINT_MAP,
    ):
        self.model_id = model_id
        self.device = device
        self.robot_to_pi_joint_map = robot_to_pi_joint_map

        self.policy = PI05Policy.from_pretrained(model_id).to(device).eval()  # pyright: ignore[reportUnknownMemberType]
        preprocess, postprocess = make_pre_post_processors(
            self.policy.config,
            model_id,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )
        self.preprocess_fn = cast(Callable[[Sample], dict[str, object]], preprocess)
        self.postprocess_fn = cast(Callable[[torch.Tensor], torch.Tensor], postprocess)

        self.expected_pi_image_keys = get_pi_image_keys(self.policy)
        self.camera_mapping = build_camera_mapping(self.expected_pi_image_keys, dataset_to_pi_camera_map)

    def predict_action(
        self,
        images: Mapping[str, object],
        joint_positions: TensorOrSeq,
        task: str,
    ) -> torch.Tensor:
        sample: Sample = {}
        for pi_camera_key, dataset_camera_key in self.camera_mapping.items():
            if dataset_camera_key not in images:
                raise ValueError(f"Missing required image key: {dataset_camera_key}")
            sample[pi_camera_key] = images[dataset_camera_key]
        sample["task"] = task

        state = _to_1d_float_tensor(joint_positions)
        sample["observation.state"] = remap_vector_to_target_dim(
            state, self.policy.config.max_state_dim, self.robot_to_pi_joint_map
        )

        batch = self.preprocess_fn(sample)
        tensor_batch = {key: value for key, value in batch.items() if isinstance(value, torch.Tensor)}
        with torch.inference_mode():
            pi_action = self.policy.select_action(tensor_batch)
            pi_action = self.postprocess_fn(pi_action)
            robot_action = remap_pi_to_robot(pi_action, self.robot_to_pi_joint_map)

        if robot_action.ndim > 1 and robot_action.shape[0] == 1:
            return robot_action[0]
        return robot_action

pipe = PIInferencePipeline()
dataset = LeRobotDataset(DATASET_ID, video_backend=VIDEO_BACKEND)

EPISODE_INDEX = 3
NUM_STATES_TO_INFER = 500

from_idx = cast(int, dataset.meta.episodes["dataset_from_index"][EPISODE_INDEX])
to_idx = cast(int, dataset.meta.episodes["dataset_to_index"][EPISODE_INDEX])

num_available_states = max(0, to_idx - from_idx)
num_states = min(NUM_STATES_TO_INFER, num_available_states)
if num_states == 0:
    raise ValueError(f"No states available for episode {EPISODE_INDEX}")

iteration_times_ms: list[float] = []
for step_offset, dataset_idx in enumerate(range(from_idx, from_idx + num_states)):
    sample = cast(Sample, dataset[dataset_idx])
    images = {key: value for key, value in sample.items() if key.startswith("observation.images.")}
    state = sample.get("observation.state")
    task = sample.get("task")
    if not isinstance(state, torch.Tensor):
        raise TypeError("Expected sample['observation.state'] to be a torch.Tensor")
    if not isinstance(task, str):
        raise TypeError("Expected sample['task'] to be a str")

    _sync_device(pipe.device)
    t0_ns = perf_counter_ns()
    pred_action = pipe.predict_action(images, state, task)
    _sync_device(pipe.device)
    dt_ms = (perf_counter_ns() - t0_ns) / 1_000_000
    iteration_times_ms.append(dt_ms)

    print(f"[step={step_offset} idx={dataset_idx}] pred_action shape:", tuple(pred_action.shape))
    print(f"[step={step_offset} idx={dataset_idx}] iter_time_ms: {dt_ms:.3f}")
    print(pred_action)

print(
    f"timing summary (ms): n={len(iteration_times_ms)} "
    f"mean={sum(iteration_times_ms) / len(iteration_times_ms):.3f} "
    f"min={min(iteration_times_ms):.3f} "
    f"max={max(iteration_times_ms):.3f}"
)
