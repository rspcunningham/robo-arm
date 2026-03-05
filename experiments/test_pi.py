import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_pre_post_processors

# Swap this import per-policy
from lerobot.policies.pi05 import PI05Policy

model_id = "lerobot/pi05_base"
dataset_id = "rspcunningham/test"
episode_index = 0
device = torch.device("mps")

# Optional explicit camera mapping for custom datasets.
# Format: {"<dataset camera key>": "<policy camera key>"}
camera_map = {
    "observation.images.front": "observation.images.base_0_rgb",
}


def get_tensor_dim(frame: dict, key: str) -> int | None:
    value = frame.get(key)
    if isinstance(value, torch.Tensor):
        return value.shape[-1]
    return None


def build_camera_mapping(frame: dict, expected_image_keys: list[str], explicit_map: dict[str, str]):
    available_image_keys = [key for key in frame.keys() if key.startswith("observation.images.")]
    mapping = {}

    for src_key, dst_key in explicit_map.items():
        if src_key in frame and dst_key in expected_image_keys:
            mapping[dst_key] = src_key

    # Fill unmapped policy cameras by position, reusing the last camera if needed.
    if available_image_keys:
        for i, dst_key in enumerate(expected_image_keys):
            if dst_key in mapping:
                continue
            src_key = available_image_keys[min(i, len(available_image_keys) - 1)]
            mapping[dst_key] = src_key

    return mapping, available_image_keys


policy = PI05Policy.from_pretrained(model_id).to(device).eval()
preprocess, postprocess = make_pre_post_processors(
    policy.config,
    model_id,
    preprocessor_overrides={"device_processor": {"device": str(device)}},
)

# Force pyav backend to avoid torchcodec+pyav FFmpeg dylib collisions on macOS.
dataset = LeRobotDataset(dataset_id, video_backend="pyav")

from_idx = dataset.meta.episodes["dataset_from_index"][episode_index]
frame = dict(dataset[from_idx])

expected_image_keys = [
    key for key in policy.config.input_features.keys() if key.startswith("observation.images.")
]
mapping, available_image_keys = build_camera_mapping(frame, expected_image_keys, camera_map)

for dst_key, src_key in mapping.items():
    frame[dst_key] = frame[src_key]

dataset_state_dim = get_tensor_dim(frame, "observation.state")
dataset_action_dim = get_tensor_dim(frame, "action")
policy_state_dim = policy.config.max_state_dim
policy_action_dim = policy.config.max_action_dim

print("dataset:", dataset_id)
print("dataset image keys:", available_image_keys)
print("policy image keys:", expected_image_keys)
print("camera mapping:", mapping)
print("dataset state/action dims:", dataset_state_dim, dataset_action_dim)
print("policy state/action dims:", policy_state_dim, policy_action_dim)
print(
    "dof mapping note: PI05 pads/truncates action internally to max_action_dim; "
    "for your own training/eval pipeline, explicitly map joint order and units."
)

batch = preprocess(frame)
with torch.inference_mode():
    pred_action = policy.select_action(batch)
    pred_action = postprocess(pred_action)

print("pred_action shape:", tuple(pred_action.shape))
print("pred_action:", pred_action)
