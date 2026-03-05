"""Replay-backed policy: return next action from one dataset episode."""

from __future__ import annotations

from pathlib import Path
import threading

from huggingface_hub import snapshot_download
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Replay source configuration (edit directly; no environment variables).
REPO_ID = "rspcunningham/test"
EPISODE = 3
PULL = True
ROOT = Path.home() / "data" / "lerobot" / REPO_ID

if PULL:
    ROOT.mkdir(parents=True, exist_ok=True)
    snapshot_download(
        repo_id=REPO_ID,
        repo_type="dataset",
        local_dir=ROOT,
    )

DATASET = LeRobotDataset(
    REPO_ID,
    root=ROOT,
    episodes=[EPISODE],
    download_videos=False,
)

ACTIONS = []
for i, raw in enumerate(DATASET.hf_dataset["action"]):
    values = raw.tolist() if hasattr(raw, "tolist") else list(raw)
    if len(values) < 4:
        raise RuntimeError(f"Invalid action length at frame {i}: {len(values)}")
    ACTIONS.append([float(v) for v in values[:4]])

if not ACTIONS:
    raise RuntimeError(f"No actions found in {REPO_ID} episode {EPISODE}")

LAST_ACTION = list(ACTIONS[-1])
INDEX = 0
LOCK = threading.Lock()


def policy(images_b64: list[str], joints: list[float], task: str) -> list[float]:
    del images_b64, joints, task
    global INDEX
    with LOCK:
        if INDEX < len(ACTIONS):
            action = ACTIONS[INDEX]
            INDEX += 1
            return list(action)
        return list(LAST_ACTION)
