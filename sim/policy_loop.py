"""Sim-to-policy-server control loop.

Replaces the Pi's policy_client.py in simulation. Same HTTP protocol:
renders camera image from MuJoCo, POSTs observation to the policy server,
and applies the returned action as joint targets.

Usage:
    uv run python policy_loop.py                          # headless
    uv run mjpython policy_loop.py --viewer               # with 3D viewer (mjpython required on macOS)
    uv run python policy_loop.py --url http://host:8000/predict
"""

import argparse
import json
import time
import urllib.error
import urllib.request

import mujoco
import mujoco.viewer

from env import JOINT_NAMES, RoArmSimEnv

DEFAULT_POLICY_URL = "http://127.0.0.1:8000/predict"
MAX_RATE_HZ = 20.0
DEFAULT_TIMEOUT_SEC = 2.0


def fetch_action(url: str, observation: dict, timeout: float) -> list[float]:
    """POST observation to the policy server and return the action."""
    payload = json.dumps(observation).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        data = json.load(response)

    action = data.get("action")
    expected_len = len(JOINT_NAMES)
    if not isinstance(action, list) or len(action) != expected_len:
        raise RuntimeError(f"Policy server returned invalid action: {data}")
    return [float(v) for v in action]


def run_headless(env: RoArmSimEnv, url: str, rate_hz: float, timeout: float):
    """Run the policy loop without visualization."""
    period = 1.0 / rate_hz
    step = 0
    print(f"Policy loop started (headless) — {url}")

    while True:
        t0 = time.monotonic()
        obs = env.observe()

        try:
            action = fetch_action(url, obs, timeout)
        except Exception as exc:
            print(f"[step {step}] Policy request failed: {exc}")
            time.sleep(period)
            step += 1
            continue

        env.step(action)
        pos = env.get_joint_positions()
        tcp = env.get_tcp_position()
        print(f"[step {step}] action={[f'{a:.3f}' for a in action]}  "
              f"pos={[f'{p:.3f}' for p in pos]}  "
              f"tcp=[{tcp[0]:.3f}, {tcp[1]:.3f}, {tcp[2]:.3f}]")

        elapsed = time.monotonic() - t0
        remaining = period - elapsed
        if remaining > 0:
            time.sleep(remaining)
        step += 1


def run_with_viewer(env: RoArmSimEnv, url: str, rate_hz: float, timeout: float):
    """Run the policy loop with MuJoCo interactive viewer."""
    period = 1.0 / rate_hz
    step = 0
    print(f"Policy loop started (viewer) — {url}")

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        while viewer.is_running():
            t0 = time.monotonic()
            obs = env.observe()

            try:
                action = fetch_action(url, obs, timeout)
            except Exception as exc:
                print(f"[step {step}] Policy request failed: {exc}")
                time.sleep(period)
                step += 1
                continue

            # Apply action
            for i, aid in enumerate(env._actuator_ids):
                env.data.ctrl[aid] = action[i]
            for _ in range(env.substeps):
                mujoco.mj_step(env.model, env.data)

            viewer.sync()

            pos = env.get_joint_positions()
            tcp = env.get_tcp_position()
            print(f"[step {step}] action={[f'{a:.3f}' for a in action]}  "
                  f"pos={[f'{p:.3f}' for p in pos]}  "
                  f"tcp=[{tcp[0]:.3f}, {tcp[1]:.3f}, {tcp[2]:.3f}]")

            elapsed = time.monotonic() - t0
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)
            step += 1


def main():
    parser = argparse.ArgumentParser(description="Sim-to-policy-server control loop")
    parser.add_argument("--url", default=DEFAULT_POLICY_URL,
                        help=f"Policy server URL (default: {DEFAULT_POLICY_URL})")
    parser.add_argument("--rate-hz", type=float, default=MAX_RATE_HZ,
                        help=f"Control rate in Hz (default: {MAX_RATE_HZ})")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_SEC,
                        help=f"HTTP timeout in seconds (default: {DEFAULT_TIMEOUT_SEC})")
    parser.add_argument("--viewer", action="store_true",
                        help="Launch interactive 3D viewer")
    args = parser.parse_args()

    env = RoArmSimEnv()
    env.reset()

    try:
        if args.viewer:
            run_with_viewer(env, args.url, args.rate_hz, args.timeout)
        else:
            run_headless(env, args.url, args.rate_hz, args.timeout)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        env.close()


if __name__ == "__main__":
    main()
