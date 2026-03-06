from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import torch
from lerobot.policies.pi05 import PI05Policy
from torch.utils.data import DataLoader
from transformers import AutoTokenizer

from dataloader import (
    DataLoaderConfig,
    ModelBatch,
    collate_pi05,
    make_raw_dataset,
)
from pi05_mps_loss import pi05_forward_loss_mps_safe

# Fixed script constants
CHECKPOINT_PATH = "../checkpoints/pi05_base"
DATASET_REPO_ID = "rspcunningham/test"
DATASET_ROOT = "../data"
OUTPUT_PLOT = "outputs/loss_curve.png"
DEVICE = torch.device("mps")
BATCH_SIZE = 1
CHUNK_SIZE = 50


def _to_device(batch: ModelBatch, device: torch.device) -> tuple[dict[str, torch.Tensor], torch.Tensor]:
    moved: dict[str, torch.Tensor] = {}
    for key, value in batch.model_inputs.items():
        moved[key] = value.to(device)
    return moved, batch.action_pad_mask.to(device)


def _run_loss_curve() -> tuple[list[int], list[float]]:
    cfg = DataLoaderConfig(
        repo_id=DATASET_REPO_ID,
        root=Path(DATASET_ROOT),
        episodes=None,  # always use all episodes
        chunk_size=CHUNK_SIZE,
    )

    raw_ds = make_raw_dataset(cfg)
    tokenizer = AutoTokenizer.from_pretrained(cfg.tokenizer_name)
    quantile_stats = raw_ds.ds.meta.stats

    def _collate(samples):
        return collate_pi05(
            samples,
            cfg,
            tokenizer=tokenizer,
            quantile_stats=quantile_stats,
        )

    loader = DataLoader(
        raw_ds,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=0,
        collate_fn=_collate,
    )

    model = PI05Policy.from_pretrained(CHECKPOINT_PATH).to(DEVICE)
    model.eval()

    steps: list[int] = []
    losses: list[float] = []

    with torch.no_grad():
        for step, batch in enumerate(loader):
            # DataLoader returns our collate output directly.
            model_batch = batch
            if not isinstance(model_batch, ModelBatch):
                raise TypeError(f"Expected ModelBatch from collate_fn, got {type(model_batch)}")

            forward_batch, action_pad_mask = _to_device(model_batch, DEVICE)
            loss, _ = pi05_forward_loss_mps_safe(
                model,
                forward_batch,
                action_pad_mask=action_pad_mask,
                reduction="mean",
            )

            steps.append(step)
            losses.append(float(loss.item()))
            print(f"step={step:04d} loss={losses[-1]:.6f}")

    return steps, losses


def _plot_loss(steps: list[int], losses: list[float], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(10, 4))
    plt.plot(steps, losses, marker="o", linewidth=1.5)
    plt.title("PI0.5 Forward Loss per Step")
    plt.xlabel("Step")
    plt.ylabel("Loss")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def main() -> None:
    if not torch.backends.mps.is_available():
        raise RuntimeError("MPS is required by this script but is not available.")

    steps, losses = _run_loss_curve()
    if len(losses) == 0:
        raise RuntimeError("No losses computed.")

    out = Path(OUTPUT_PLOT)
    _plot_loss(steps, losses, out)
    print(f"\nSaved loss plot to: {out.resolve()}")
    print(
        f"loss summary: n={len(losses)} min={min(losses):.6f} max={max(losses):.6f} mean={sum(losses)/len(losses):.6f}"
    )


if __name__ == "__main__":
    main()
