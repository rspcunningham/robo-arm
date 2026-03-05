"""MuJoCo simulation environment for the RoArm-M2-S."""

import base64
import io
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

MJCF_PATH = Path(__file__).parent / "roarm.xml"
JOINT_NAMES = ["base", "shoulder", "elbow", "hand"]

# Camera image size (matches the real ESP32-CAM pipeline: 480x640)
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640


class RoArmSimEnv:
    """Simulation environment wrapping MuJoCo for the RoArm-M2-S.

    Joint angles go directly to/from the policy server with no additional
    transform — the MJCF joint axes encode the same conventions as the URDF,
    which matches what arm.py publishes after its firmware→ROS transform.
    """

    def __init__(self, render_width: int = IMAGE_WIDTH, render_height: int = IMAGE_HEIGHT):
        self.model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
        self.data = mujoco.MjData(self.model)

        self.render_width = render_width
        self.render_height = render_height

        # Renderer for camera images
        self._renderer = mujoco.Renderer(self.model, height=render_height, width=render_width)

        # Cache joint and actuator indices
        self._joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
        self._actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{n}_actuator") for n in JOINT_NAMES]
        self._tcp_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "hand_tcp")
        self._cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "esp_cam")

        # Number of physics substeps per control step (50ms control at 2ms timestep)
        self.substeps = 25

    def reset(self, qpos: list[float] | None = None) -> None:
        """Reset the simulation, optionally setting initial joint positions."""
        mujoco.mj_resetData(self.model, self.data)
        if qpos is not None:
            for i, jid in enumerate(self._joint_ids):
                addr = self.model.jnt_qposadr[jid]
                self.data.qpos[addr] = qpos[i]
                # Also set actuator targets so it holds position
                self.data.ctrl[self._actuator_ids[i]] = qpos[i]
        mujoco.mj_forward(self.model, self.data)

    def step(self, action: list[float]) -> None:
        """Apply a 4-DOF position command and advance physics."""
        for i, aid in enumerate(self._actuator_ids):
            self.data.ctrl[aid] = action[i]
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)

    def get_joint_positions(self) -> list[float]:
        """Return current joint positions in ROS convention."""
        return [float(self.data.qpos[self.model.jnt_qposadr[jid]]) for jid in self._joint_ids]

    def observe(self) -> dict:
        """Return observation dict matching the policy server's expected input.

        Returns:
            dict with 'images_b64' (list[str]) and 'joints' (list[float]).
        """
        # Render camera image
        image_b64 = self._render_camera_b64()

        # Joint state
        positions = self.get_joint_positions()
        return {
            "images_b64": [image_b64],
            "joints": positions[:len(JOINT_NAMES)],
        }

    def render(self) -> np.ndarray:
        """Render and return an RGB numpy array from the sim camera."""
        self._renderer.update_scene(self.data, camera=self._cam_id)
        return self._renderer.render().copy()

    def get_tcp_position(self) -> np.ndarray:
        """Return the 3D world position of the tool center point."""
        return self.data.site_xpos[self._tcp_site_id].copy()

    def _render_camera_b64(self) -> str:
        """Render camera view and return as base64 JPEG string."""
        rgb = self.render()
        img = Image.fromarray(rgb)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    def close(self) -> None:
        """Clean up renderer resources."""
        self._renderer.close()


if __name__ == "__main__":
    env = RoArmSimEnv()
    env.reset()
    print(f"Joint positions: {env.get_joint_positions()}")
    print(f"TCP position: {env.get_tcp_position()}")
    obs = env.observe()
    print(f"Image b64 length: {len(obs['images_b64'][0])}")
    env.close()
