"""Train a PPO policy to reach random targets with the RoArm-M2-S."""

import argparse
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import SubprocVecEnv


def make_env(seed: int):
    def _init():
        from reach.reach_env import RoArmReachEnv

        env = RoArmReachEnv()
        env.reset(seed=seed)
        return env

    return _init


def main():
    parser = argparse.ArgumentParser(description="Train PPO reach policy")
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--run-dir", type=str, default="runs/reach")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    # Parallel training envs
    train_env = SubprocVecEnv(
        [make_env(args.seed + i) for i in range(args.n_envs)]
    )

    # Single eval env
    eval_env = SubprocVecEnv([make_env(args.seed + 1000)])

    model = PPO(
        "MlpPolicy",
        train_env,
        policy_kwargs=dict(net_arch=[256, 256]),
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        verbose=1,
        seed=args.seed,
        tensorboard_log=str(run_dir / "tb"),
    )

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(run_dir / "best"),
        log_path=str(run_dir / "eval_logs"),
        eval_freq=max(10_000 // args.n_envs, 1),
        n_eval_episodes=20,
        deterministic=True,
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=max(50_000 // args.n_envs, 1),
        save_path=str(run_dir / "checkpoints"),
        name_prefix="reach",
    )

    print(f"Training PPO for {args.timesteps} timesteps with {args.n_envs} envs")
    print(f"Logs: {run_dir / 'tb'}")

    model.learn(
        total_timesteps=args.timesteps,
        callback=[eval_callback, checkpoint_callback],
        progress_bar=True,
    )

    final_path = run_dir / "final_model"
    model.save(str(final_path))
    print(f"Final model saved to {final_path}")

    train_env.close()
    eval_env.close()


if __name__ == "__main__":
    main()
