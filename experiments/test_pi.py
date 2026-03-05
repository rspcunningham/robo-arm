import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_pre_post_processors

# Swap this import per-policy
from lerobot.policies.pi05 import PI05Policy

# load a policy
model_id = "lerobot/pi05_base"  # <- swap checkpoint
device = torch.device("mps")

policy = PI05Policy.from_pretrained(model_id).to(device).eval()

preprocess, postprocess = make_pre_post_processors(
    policy.config,
    model_id,
    preprocessor_overrides={"device_processor": {"device": str(device)}},
)
# Force pyav backend to avoid torchcodec+pyav FFmpeg dylib collisions on macOS.
dataset = LeRobotDataset("lerobot/libero", video_backend="pyav")

# pick an episode
episode_index = 0

# each episode corresponds to a contiguous range of frame indices
from_idx = dataset.meta.episodes["dataset_from_index"][episode_index]
to_idx   = dataset.meta.episodes["dataset_to_index"][episode_index]

# get a single frame from that episode (e.g. the first frame)
frame_index = from_idx
frame = dict(dataset[frame_index])

expected_image_keys = [
    key for key in policy.config.input_features.keys() if key.startswith("observation.images.")
]
available_image_keys = [key for key in frame.keys() if key.startswith("observation.images.")]

if expected_image_keys and available_image_keys:
    # PI0.5 expects named cameras (base/left/right wrist). Some datasets provide
    # generic names (e.g. image/image2), so remap by position and repeat the last
    # available camera if fewer views are present.
    for i, dst_key in enumerate(expected_image_keys):
        src_key = available_image_keys[min(i, len(available_image_keys) - 1)]
        frame[dst_key] = frame[src_key]

batch = preprocess(frame)
with torch.inference_mode():
    pred_action = policy.select_action(batch)
    # use your policy postprocess, this post process the action
    # for instance unnormalize the actions, detokenize it etc..
    pred_action = postprocess(pred_action)

print("pred_action:", pred_action)
