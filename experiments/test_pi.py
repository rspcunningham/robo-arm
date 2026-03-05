import torch
from typing import Callable, cast

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.pi05 import PI05Policy

MODEL_ID = "lerobot/pi05_base"
DATASET_ID = "rspcunningham/test"
EPISODE_INDEX = 0
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


def tensor_last_dim(sample: Sample, key: str) -> int | None:
    value = sample.get(key)
    if isinstance(value, torch.Tensor):
        return value.shape[-1]
    return None


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
    sample: Sample, expected_pi_image_keys: list[str], dataset_to_pi_camera_map: dict[str, str]
) -> tuple[dict[str, str], list[str]]:
    available_image_keys = [key for key in sample.keys() if key.startswith("observation.images.")]
    resolved_mapping: dict[str, str] = {}
    for dataset_camera_key, pi_camera_key in dataset_to_pi_camera_map.items():
        if dataset_camera_key in sample and pi_camera_key in expected_pi_image_keys:
            resolved_mapping[pi_camera_key] = dataset_camera_key
    return resolved_mapping, available_image_keys


def main() -> None:
    policy = PI05Policy.from_pretrained(MODEL_ID).to(DEVICE).eval()  # pyright: ignore[reportUnknownMemberType]
    preprocess, postprocess = make_pre_post_processors(
        policy.config,
        MODEL_ID,
        preprocessor_overrides={"device_processor": {"device": str(DEVICE)}},
    )
    preprocess_fn = cast(Callable[[Sample], dict[str, object]], preprocess)
    postprocess_fn = cast(Callable[[torch.Tensor], torch.Tensor], postprocess)

    dataset = LeRobotDataset(DATASET_ID, video_backend=VIDEO_BACKEND)
    from_idx = cast(int, dataset.meta.episodes["dataset_from_index"][EPISODE_INDEX])
    sample = cast(Sample, dataset[from_idx])

    expected_pi_image_keys = get_pi_image_keys(policy)
    camera_mapping, available_dataset_image_keys = build_camera_mapping(
        sample, expected_pi_image_keys, CAMERA_MAP_DATASET_TO_PI
    )
    for pi_camera_key, dataset_camera_key in camera_mapping.items():
        sample[pi_camera_key] = sample[dataset_camera_key]

    state_dim_raw = tensor_last_dim(sample, "observation.state")
    action_dim_raw = tensor_last_dim(sample, "action")

    state_tensor = sample.get("observation.state")
    if isinstance(state_tensor, torch.Tensor):
        sample["observation.state"] = remap_vector_to_target_dim(
            state_tensor, policy.config.max_state_dim, ROBOT_TO_PI_JOINT_MAP
        )
    action_tensor = sample.get("action")
    if isinstance(action_tensor, torch.Tensor):
        sample["action"] = remap_vector_to_target_dim(
            action_tensor, policy.config.max_action_dim, ROBOT_TO_PI_JOINT_MAP
        )

    state_dim_mapped = tensor_last_dim(sample, "observation.state")
    action_dim_mapped = tensor_last_dim(sample, "action")

    print("dataset:", DATASET_ID)
    print("dataset image keys:", available_dataset_image_keys)
    print("policy image keys:", expected_pi_image_keys)
    print("camera mapping:", camera_mapping)
    print("masked cameras:", [key for key in expected_pi_image_keys if key not in camera_mapping])
    print("robot->pi joint map:", ROBOT_TO_PI_JOINT_MAP)
    print("dataset state/action dims (raw):", state_dim_raw, action_dim_raw)
    print("dataset state/action dims (mapped):", state_dim_mapped, action_dim_mapped)
    print("policy state/action dims:", policy.config.max_state_dim, policy.config.max_action_dim)

    batch = preprocess_fn(sample)
    tensor_batch = {key: value for key, value in batch.items() if isinstance(value, torch.Tensor)}
    with torch.inference_mode():
        pi_action = policy.select_action(tensor_batch)
        pi_action = postprocess_fn(pi_action)
        robot_action = remap_pi_to_robot(pi_action, ROBOT_TO_PI_JOINT_MAP)

    print("pi_action shape:", tuple(pi_action.shape))
    print("pi_action:", pi_action)
    print("robot_action shape:", tuple(robot_action.shape))
    print("robot_action:", robot_action)


if __name__ == "__main__":
    main()
