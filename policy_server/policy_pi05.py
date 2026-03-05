from __future__ import annotations

import base64
from collections.abc import Sequence
import io
from typing import Callable, cast

import numpy as np
from PIL import Image
import torch

from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.pi05 import PI05Policy

MODEL_ID = "lerobot/pi05_base"
DEVICE = torch.device("mps")
ACTION_DIM = 4

# Robot joint index -> PI0.5 joint index.
ROBOT_TO_PI_JOINT_MAP = {
    0: 0,
    1: 1,
    2: 2,
    3: 6,
}

Sample = dict[str, object]
TensorOrSeq = torch.Tensor | Sequence[float]


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


def _to_1d_float_tensor(values: TensorOrSeq) -> torch.Tensor:
    if isinstance(values, torch.Tensor):
        tensor = values.to(dtype=torch.float32)
    else:
        tensor = torch.as_tensor(values, dtype=torch.float32)
    return tensor.reshape(-1)


def _decode_jpeg_b64(image_b64: str) -> np.ndarray:
    raw = base64.b64decode(image_b64)
    with Image.open(io.BytesIO(raw)) as image:
        rgb = image.convert("RGB")
        return np.asarray(rgb, dtype=np.uint8)


def _masked_image_like(reference_image: np.ndarray) -> np.ndarray:
    return np.zeros_like(reference_image, dtype=np.uint8)


class PIInferencePipeline:
    model_id: str
    device: torch.device
    robot_to_pi_joint_map: dict[int, int]
    policy: PI05Policy
    preprocess_fn: Callable[[Sample], dict[str, object]]
    postprocess_fn: Callable[[torch.Tensor], torch.Tensor]
    expected_pi_image_keys: list[str]

    def __init__(
        self,
        model_id: str = MODEL_ID,
        device: torch.device = DEVICE,
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
        if not self.expected_pi_image_keys:
            raise RuntimeError("PI0.5 policy has no expected image keys in config.input_features")

    def predict_action(
        self,
        images_b64: Sequence[str],
        joints: TensorOrSeq,
        task: str,
    ) -> list[float]:
        if len(images_b64) != 1:
            raise ValueError(f"Expected exactly 1 image(s) for base_0_rgb, got {len(images_b64)}")

        base_image = _decode_jpeg_b64(images_b64[0])
        sample: Sample = {}
        for pi_camera_key in self.expected_pi_image_keys:
            if "base_0_rgb" in pi_camera_key:
                sample[pi_camera_key] = base_image
            elif "left_wrist_0_rgb" in pi_camera_key or "right_wrist_0_rgb" in pi_camera_key:
                sample[pi_camera_key] = _masked_image_like(base_image)
            else:
                raise ValueError(f"Unsupported camera key for single-image mode: {pi_camera_key}")
        sample["task"] = task

        state = _to_1d_float_tensor(joints)
        sample["observation.state"] = remap_vector_to_target_dim(
            state, self.policy.config.max_state_dim, self.robot_to_pi_joint_map
        )

        batch = self.preprocess_fn(sample)
        tensor_batch: dict[str, torch.Tensor] = {}
        for key, value in batch.items():
            if isinstance(value, torch.Tensor):
                tensor_batch[key] = value
                continue
            if key in self.expected_pi_image_keys and isinstance(value, np.ndarray):
                image_np = np.array(value, copy=True)
                image_tensor = torch.from_numpy(image_np)
                if image_tensor.ndim == 3 and image_tensor.shape[-1] == 3:
                    image_tensor = image_tensor.permute(2, 0, 1).unsqueeze(0)
                elif image_tensor.ndim == 3:
                    image_tensor = image_tensor.unsqueeze(0)
                image_tensor = image_tensor.to(dtype=torch.float32)
                if image_tensor.numel() > 0 and image_tensor.max() > 1.0:
                    image_tensor = image_tensor / 255.0
                tensor_batch[key] = image_tensor
        with torch.inference_mode():
            pi_action = self.policy.select_action(tensor_batch)
            pi_action = self.postprocess_fn(pi_action)
            robot_action = remap_pi_to_robot(pi_action, self.robot_to_pi_joint_map)

        if robot_action.ndim > 1 and robot_action.shape[0] == 1:
            robot_action = robot_action[0]

        action = [float(value) for value in robot_action.reshape(-1).tolist()]
        if len(action) < ACTION_DIM:
            raise RuntimeError(f"Policy returned {len(action)} values, expected at least {ACTION_DIM}")
        return action[:ACTION_DIM]


_PIPE = PIInferencePipeline()


def _get_pipe() -> PIInferencePipeline:
    return _PIPE


def inference(images_b64: list[str], joints: list[float], task: str) -> list[float]:
    return _get_pipe().predict_action(images_b64, joints, task)
