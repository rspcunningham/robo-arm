from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from torch import Tensor
from torch.utils.data import Dataset
from transformers import AutoTokenizer
from transformers.tokenization_utils_base import PreTrainedTokenizerBase

OBS_LANGUAGE_ATTENTION_MASK = "observation.language.attention_mask"
OBS_LANGUAGE_TOKENS = "observation.language.tokens"


@dataclass(frozen=True)
class DataLoaderConfig:
    repo_id: str
    root: Path
    episodes: list[int] | None = None
    video_backend: str = "pyav"
    chunk_size: int = 50
    max_state_dim: int = 32
    max_action_dim: int = 32
    camera_source_key: str = "observation.images.front"
    camera_target_key: str = "observation.images.base_0_rgb"
    robot_to_pi_joint_map: dict[int, int] | None = None
    tokenizer_name: str = "google/paligemma-3b-pt-224"
    tokenizer_max_length: int = 200
    apply_quantile_norm: bool = True

    def __post_init__(self) -> None:
        if self.robot_to_pi_joint_map is None:
            object.__setattr__(self, "robot_to_pi_joint_map", {0: 0, 1: 1, 2: 2, 3: 6})


@dataclass
class RawSample:
    image_front: Tensor
    state: Tensor
    action_chunk: Tensor
    action_pad_mask: Tensor
    task: str
    episode_index: int
    frame_index: int
    dataset_index: int


@dataclass
class ModelBatch:
    model_inputs: dict[str, Tensor]
    action_pad_mask: Tensor
    metadata: dict[str, Any]

    def as_forward_batch(self) -> dict[str, Tensor]:
        return self.model_inputs


class LeRobotWindowDataset(Dataset[RawSample]):
    """Model-agnostic windowed dataset.

    Returns one frame input + fixed-length future action chunk with pad mask.
    """

    def __init__(self, cfg: DataLoaderConfig):
        self.cfg = cfg
        self.ds = LeRobotDataset(
            cfg.repo_id,
            root=cfg.root,
            episodes=cfg.episodes,
            video_backend=cfg.video_backend,
            force_cache_sync=True,
        )

        # Build a direct index map for stable random access.
        self.indices = list(range(len(self.ds)))
        self.episode_index_col = self.ds.hf_dataset["episode_index"]

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, i: int) -> RawSample:
        idx = self.indices[i]
        sample = self.ds[idx]

        image_front = sample[self.cfg.camera_source_key]
        state = sample["observation.state"].to(torch.float32)
        action_chunk, action_pad_mask = self._build_action_chunk(idx)

        task_val = sample.get("task", "")
        task = task_val if isinstance(task_val, str) else ""

        return RawSample(
            image_front=image_front.to(torch.float32),
            state=state,
            action_chunk=action_chunk,
            action_pad_mask=action_pad_mask,
            task=task,
            episode_index=int(sample["episode_index"]),
            frame_index=int(sample["frame_index"]),
            dataset_index=int(sample["index"]),
        )

    def _build_action_chunk(self, idx: int) -> tuple[Tensor, Tensor]:
        """Returns (chunk[T, action_dim], pad_mask[T]) padded at episode tail."""
        chunk_actions: list[Tensor] = []
        mask = torch.zeros(self.cfg.chunk_size, dtype=torch.bool)
        episode_index = int(self.episode_index_col[idx])

        for t in range(self.cfg.chunk_size):
            future_idx = idx + t
            if future_idx >= len(self.ds):
                break
            if int(self.episode_index_col[future_idx]) != episode_index:
                break
            future_sample = self.ds[future_idx]
            chunk_actions.append(future_sample["action"].to(torch.float32))
            mask[t] = True

        if not chunk_actions:
            action_dim = int(self.ds[0]["action"].shape[-1])
            chunk = torch.zeros(self.cfg.chunk_size, action_dim, dtype=torch.float32)
            return chunk, mask

        chunk = torch.stack(chunk_actions, dim=0)
        action_dim = int(chunk.shape[-1])
        if chunk.shape[0] < self.cfg.chunk_size:
            pad = torch.zeros(self.cfg.chunk_size - chunk.shape[0], action_dim, dtype=chunk.dtype)
            chunk = torch.cat([chunk, pad], dim=0)

        return chunk, mask


def make_raw_dataset(cfg: DataLoaderConfig) -> LeRobotWindowDataset:
    return LeRobotWindowDataset(cfg)


def collate_pi05(
    samples: list[RawSample],
    cfg: DataLoaderConfig,
    *,
    tokenizer: PreTrainedTokenizerBase | None = None,
    quantile_stats: Mapping[str, Mapping[str, Any]] | None = None,
) -> ModelBatch:
    """PI0.5-specific adapter.

    Converts generic RawSample -> model-ready batch for PI05Policy.forward.
    """
    if len(samples) == 0:
        raise ValueError("collate_pi05 received an empty sample list")

    tok = tokenizer or AutoTokenizer.from_pretrained(cfg.tokenizer_name)

    images = torch.stack([s.image_front for s in samples], dim=0).to(torch.float32)
    state_raw = torch.stack([s.state for s in samples], dim=0).to(torch.float32)
    action_raw = torch.stack([s.action_chunk for s in samples], dim=0).to(torch.float32)
    action_pad_mask = torch.stack([s.action_pad_mask for s in samples], dim=0)

    # Optional quantile normalization in original robot space (4-d here),
    # then remap into PI sparse 32-d space.
    state_norm = state_raw
    action_norm = action_raw
    if cfg.apply_quantile_norm and quantile_stats is not None:
        state_norm = _quantile_normalize(state_norm, quantile_stats, key="observation.state")
        action_norm = _quantile_normalize(action_norm, quantile_stats, key="action")

    # PI0.5 mapping: robot joints into sparse 32-d PI space.
    state_mapped = _remap_and_pad_last_dim(
        state_norm,
        target_dim=cfg.max_state_dim,
        source_to_target_map=cfg.robot_to_pi_joint_map or {},
    )
    action_mapped = _remap_and_pad_last_dim(
        action_norm,
        target_dim=cfg.max_action_dim,
        source_to_target_map=cfg.robot_to_pi_joint_map or {},
    )

    prompts = [_build_pi05_prompt(sample.task, state_mapped[i]) for i, sample in enumerate(samples)]
    tok_out = tok(
        prompts,
        max_length=cfg.tokenizer_max_length,
        padding="max_length",
        truncation=True,
        return_tensors="pt",
    )

    # NOTE:
    # - We only provide base camera key.
    # - PI0.5 forward path auto-masks missing wrist camera keys.
    model_inputs = {
        cfg.camera_target_key: images,
        OBS_LANGUAGE_TOKENS: tok_out["input_ids"],
        OBS_LANGUAGE_ATTENTION_MASK: tok_out["attention_mask"],
        "action": action_mapped,
    }

    metadata = {
        "episode_index": torch.tensor([s.episode_index for s in samples], dtype=torch.long),
        "frame_index": torch.tensor([s.frame_index for s in samples], dtype=torch.long),
        "dataset_index": torch.tensor([s.dataset_index for s in samples], dtype=torch.long),
        "task": [s.task for s in samples],
    }
    return ModelBatch(model_inputs=model_inputs, action_pad_mask=action_pad_mask, metadata=metadata)


def _remap_and_pad_last_dim(
    source: Tensor,
    *,
    target_dim: int,
    source_to_target_map: dict[int, int],
) -> Tensor:
    out = torch.zeros(*source.shape[:-1], target_dim, dtype=source.dtype)
    for src_idx, tgt_idx in source_to_target_map.items():
        if src_idx < source.shape[-1] and tgt_idx < target_dim:
            out[..., tgt_idx] = source[..., src_idx]
    return out


def _quantile_normalize(
    values: Tensor,
    stats: Mapping[str, Mapping[str, Any]],
    *,
    key: str,
    eps: float = 1e-8,
) -> Tensor:
    """Maps values to [-1, 1] using q01/q99, matching PI0.5 data conventions."""
    if key not in stats:
        return values

    q01 = torch.as_tensor(stats[key]["q01"], dtype=values.dtype)
    q99 = torch.as_tensor(stats[key]["q99"], dtype=values.dtype)
    denom = torch.where((q99 - q01) == 0, torch.tensor(eps, dtype=values.dtype), (q99 - q01))
    return 2.0 * (values - q01) / denom - 1.0


def _build_pi05_prompt(task: str, normalized_padded_state: Tensor) -> str:
    cleaned = task.strip().replace("_", " ").replace("\n", " ")
    state_np = normalized_padded_state.detach().cpu().numpy().astype(np.float32, copy=False)
    discretized = np.digitize(state_np, bins=np.linspace(-1, 1, 256 + 1)[:-1]) - 1
    state_str = " ".join(map(str, discretized.tolist()))
    return f"Task: {cleaned}, State: {state_str};\nAction: "


if __name__ == "__main__":
    # Lightweight smoke test:
    #   uv run dataloader.py
    cfg = DataLoaderConfig(
        repo_id="rspcunningham/test",
        root=Path("../data"),
        episodes=[0],
        chunk_size=50,
    )

    raw_ds = make_raw_dataset(cfg)
    print(f"raw dataset length: {len(raw_ds)}")

    sample0 = raw_ds[0]
    print("raw sample")
    print(f"  image_front: shape={tuple(sample0.image_front.shape)} dtype={sample0.image_front.dtype}")
    print(f"  state: shape={tuple(sample0.state.shape)} dtype={sample0.state.dtype}")
    print(f"  action_chunk: shape={tuple(sample0.action_chunk.shape)} dtype={sample0.action_chunk.dtype}")
    print(f"  action_pad_mask true_count={int(sample0.action_pad_mask.sum().item())}/{cfg.chunk_size}")
    print(f"  task: {sample0.task!r}")

    batch_samples = [raw_ds[i] for i in range(min(2, len(raw_ds)))]
    model_batch = collate_pi05(
        batch_samples,
        cfg,
        quantile_stats=raw_ds.ds.meta.stats,
    )

    print("\nmodel input keys")
    for key, value in model_batch.model_inputs.items():
        print(f"  {key}: shape={tuple(value.shape)} dtype={value.dtype}")
    print(
        f"  action_pad_mask: shape={tuple(model_batch.action_pad_mask.shape)} dtype={model_batch.action_pad_mask.dtype}"
    )
