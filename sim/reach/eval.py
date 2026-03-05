"""Evaluate a trained reach policy — headless stats or MuJoCo viewer."""

import argparse

import numpy as np
from stable_baselines3 import PPO

from reach.reach_env import RoArmReachEnv


def eval_headless(model: PPO, n_episodes: int = 50, seed: int = 0) -> None:
    """Run episodes headless and print stats."""
    env = RoArmReachEnv()
    successes = 0
    distances = []

    for ep in range(n_episodes):
        obs, _ = env.reset(seed=seed + ep)
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
        distances.append(info["distance"])
        if info["success"]:
            successes += 1

    env.close()

    rate = successes / n_episodes * 100
    mean_dist = np.mean(distances)
    print(f"Episodes: {n_episodes}")
    print(f"Success rate: {rate:.1f}% ({successes}/{n_episodes})")
    print(f"Mean final distance: {mean_dist:.4f} m")


def eval_viewer(model: PPO, n_episodes: int = 10, seed: int = 0) -> None:
    """Run episodes in the MuJoCo interactive viewer."""
    import time

    import mujoco.viewer

    env = RoArmReachEnv()
    dt = 1.0 / 20  # 20 Hz control rate

    with mujoco.viewer.launch_passive(env.sim.model, env.sim.data) as viewer:
        for ep in range(n_episodes):
            obs, _ = env.reset(seed=seed + ep)
            viewer.sync()
            time.sleep(0.5)  # pause before each episode

            done = False
            step = 0
            while not done and viewer.is_running():
                action, _ = model.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = env.step(action)
                done = terminated or truncated
                step += 1
                viewer.sync()
                time.sleep(dt)

            status = "SUCCESS" if info["success"] else "FAIL"
            print(
                f"Episode {ep + 1}: {status} | "
                f"steps={step} | dist={info['distance']:.4f}"
            )
            time.sleep(1.0)  # pause between episodes

            if not viewer.is_running():
                break

    env.close()


def main():
    parser = argparse.ArgumentParser(description="Evaluate reach policy")
    parser.add_argument("model_path", help="Path to saved model zip")
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--viewer", action="store_true",
        help="Show in MuJoCo viewer (use mjpython on macOS)",
    )
    args = parser.parse_args()

    model = PPO.load(args.model_path)
    print(f"Loaded model from {args.model_path}")

    if args.viewer:
        eval_viewer(model, n_episodes=args.episodes, seed=args.seed)
    else:
        eval_headless(model, n_episodes=args.episodes, seed=args.seed)


if __name__ == "__main__":
    main()
