"""Interactive demo and smoke test for the RoArm-M2-S MuJoCo simulation.

Usage:
    uv run python demo.py              # launch interactive viewer
    uv run mjpython demo.py --test-range # sweep each joint (needs mjpython on macOS)
    uv run python demo.py --save-image test.png  # render camera view to file
"""

import argparse
import time

import mujoco
import mujoco.viewer
from PIL import Image

from env import JOINT_NAMES, MJCF_PATH, RoArmSimEnv


def launch_viewer():
    """Launch the MuJoCo interactive viewer."""
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mujoco.viewer.launch(model, data)


def test_range():
    """Sweep each joint through its limits one at a time, shown in the viewer.

    Requires running with mjpython on macOS:
        uv run mjpython demo.py --test-range
    """
    env = RoArmSimEnv()
    env.reset()

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        for i, name in enumerate(JOINT_NAMES):
            if not viewer.is_running():
                break
            jid = env._joint_ids[i]
            lo = env.model.jnt_range[jid, 0]
            hi = env.model.jnt_range[jid, 1]
            print(f"Sweeping {name}: {lo:.3f} -> {hi:.3f}")

            # Sweep from low to high
            for s in range(80):
                if not viewer.is_running():
                    break
                frac = s / 79
                target = lo + frac * (hi - lo)
                action = env.get_joint_positions()
                action[i] = target
                env.step(action)
                viewer.sync()
                time.sleep(0.02)

            # Sweep back to center
            for s in range(50):
                if not viewer.is_running():
                    break
                action = env.get_joint_positions()
                action[i] = 0.0
                env.step(action)
                viewer.sync()
                time.sleep(0.02)

            pos = env.get_joint_positions()
            print(f"  Final positions: {[f'{p:.3f}' for p in pos]}")

    print("Joint range test complete!")
    env.close()


def save_image(path: str):
    """Render the camera view and save to a file."""
    env = RoArmSimEnv()
    env.reset()

    # Let the sim settle for a moment
    for _ in range(10):
        env.step([0.0, 0.0, 0.0, 0.0])

    rgb = env.render()
    img = Image.fromarray(rgb)
    img.save(path)
    print(f"Saved {rgb.shape[1]}x{rgb.shape[0]} image to {path}")
    env.close()


def main():
    parser = argparse.ArgumentParser(description="RoArm-M2-S MuJoCo demo")
    parser.add_argument("--test-range", action="store_true", help="Sweep each joint through its limits (run with mjpython)")
    parser.add_argument("--save-image", metavar="PATH", help="Render camera view to file")
    args = parser.parse_args()

    if args.test_range:
        test_range()
    elif args.save_image:
        save_image(args.save_image)
    else:
        launch_viewer()


if __name__ == "__main__":
    main()
