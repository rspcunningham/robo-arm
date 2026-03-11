"""Collect rollouts from a trained reach policy.

Runs the PPO policy in sim, recording (camera image, joint state, action)
for downstream VLA training.

Preview mode (--preview): runs one episode, dumps frames as PNGs for sanity checking.
Full mode: collects into a LeRobot v3 dataset.

Usage:
    cd sim

    # Quick sanity check — dumps frames to preview/
    uv run --extra rl python reach/collect.py runs/best_model.zip --preview

    # Full collection
    uv run --extra rl python reach/collect.py runs/best_model.zip \
        --repo-id my-org/reach-sim --episodes 200 --task "reach the red block"
"""

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from stable_baselines3 import PPO

from reach.reach_env import RoArmReachEnv

JOINT_NAMES = ["base", "shoulder", "elbow", "hand"]

LEROBOT_FEATURES = {
    "observation.images.front": {
        "dtype": "video",
        "shape": [480, 640, 3],
        "names": ["height", "width", "channels"],
    },
    "observation.state": {
        "dtype": "float32",
        "shape": (4,),
        "names": JOINT_NAMES,
    },
    "action": {
        "dtype": "float32",
        "shape": (4,),
        "names": JOINT_NAMES,
    },
}

FPS = 20  # matches sim control rate


def preview(model: PPO, env: RoArmReachEnv, task: str, out_dir: Path, seed: int):
    """Run one successful episode and dump annotated frames as PNGs."""
    out_dir.mkdir(parents=True, exist_ok=True)
    attempt = 0

    while True:
        obs, _ = env.reset(seed=seed + attempt)
        attempt += 1
        done = False
        step = 0
        frames = []

        while not done:
            rgb = env.sim.render()
            joints = np.array(env.sim.get_joint_positions(), dtype=np.float32)

            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            step += 1

            action_joints = np.array(
                [env.sim.data.ctrl[aid] for aid in env._ctrl_actuator_ids]
                + [0.0],
                dtype=np.float32,
            )

            frames.append({
                "rgb": rgb,
                "state": joints,
                "action": action_joints,
                "dist": info["distance"],
                "step": step,
            })

        if info["success"]:
            break
        print(f"  attempt {attempt}: FAIL (dist={info['distance']:.4f}), retrying...")

    for f in frames:
        img = Image.fromarray(f["rgb"])
        draw = ImageDraw.Draw(img)
        state_str = " ".join(f"{v:+.2f}" for v in f["state"])
        action_str = " ".join(f"{v:+.2f}" for v in f["action"])
        lines = [
            f"step {f['step']:03d}  dist={f['dist']:.3f}",
            f"state:  [{state_str}]",
            f"action: [{action_str}]",
        ]
        y = 5
        for line in lines:
            draw.text((5, y), line, fill=(255, 255, 255))
            y += 14
        img.save(out_dir / f"frame_{f['step']:03d}.png")

    print(f"Saved {len(frames)} frames to {out_dir}/")
    print(f"  episode solved in {len(frames)} steps (attempt {attempt}), final dist={info['distance']:.4f}")


def get_dataset(repo_id: str, root: Path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root.mkdir(parents=True, exist_ok=True)
    return LeRobotDataset.create(
        repo_id=repo_id,
        fps=FPS,
        features=LEROBOT_FEATURES,
        root=root,
        robot_type="so100",
        use_videos=True,
        vcodec="h264",
    )


def collect(model: PPO, dataset, env: RoArmReachEnv, task: str, n_episodes: int, seed: int):
    successes = 0
    attempts = 0

    while successes < n_episodes:
        obs, _ = env.reset(seed=seed + attempts)
        attempts += 1
        done = False
        steps = 0
        frames = []

        while not done:
            rgb = env.sim.render()
            image = Image.fromarray(rgb)
            joints = np.array(env.sim.get_joint_positions(), dtype=np.float32)

            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            steps += 1

            action_joints = np.array(
                [env.sim.data.ctrl[aid] for aid in env._ctrl_actuator_ids]
                + [0.0],  # hand fixed
                dtype=np.float32,
            )

            frames.append({
                "task": task,
                "observation.images.front": image,
                "observation.state": joints,
                "action": action_joints,
            })

        if not info["success"]:
            print(f"  attempt {attempts}: FAIL (dist={info['distance']:.4f}), skipping")
            continue

        for frame in frames:
            dataset.add_frame(frame)
        dataset.save_episode()
        successes += 1
        print(f"  ep {successes}/{n_episodes} (attempt {attempts}): steps={steps} | dist={info['distance']:.4f}")

    return successes, attempts


def main():
    parser = argparse.ArgumentParser(description="Collect reach rollouts")
    parser.add_argument("model_path", help="Path to trained PPO model zip")
    parser.add_argument("--preview", action="store_true", help="Dump one episode as PNGs for sanity check")
    parser.add_argument("--preview-dir", default="preview", help="Output dir for preview frames")
    parser.add_argument("--repo-id", default=None, help="Dataset repo id (required for full collection)")
    parser.add_argument("--data-dir", default="data", help="Local dataset root (default: data/)")
    parser.add_argument("--episodes", type=int, default=200, help="Number of episodes to collect")
    parser.add_argument("--task", default="reach the red block", help="Task description string")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--push", action="store_true", help="Push dataset to HuggingFace Hub")
    parser.add_argument("--public", action="store_true", help="Make HF repo public (with --push)")
    args = parser.parse_args()

    model = PPO.load(args.model_path)
    print(f"Loaded policy from {args.model_path}")

    env = RoArmReachEnv()

    if args.preview:
        preview(model, env, args.task, Path(args.preview_dir), args.seed)
        env.close()
        return

    if not args.repo_id:
        parser.error("--repo-id is required for full collection (or use --preview)")

    root = Path(args.data_dir) / args.repo_id
    dataset = get_dataset(args.repo_id, root)
    print(f"Dataset: {root}")
    print(f"Collecting {args.episodes} episodes...")

    successes, attempts = collect(model, dataset, env, args.task, args.episodes, args.seed)
    env.close()

    print(f"\nDone: {successes} successful episodes from {attempts} attempts")
    print(f"Dataset: {dataset.num_episodes} episodes, {dataset.num_frames} frames")

    dataset.finalize()

    if args.push:
        print(f"Pushing to HF: {args.repo_id}...", end="", flush=True)
        dataset.push_to_hub(private=not args.public)
        print(" done.")


if __name__ == "__main__":
    main()
