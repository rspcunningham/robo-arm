"""Bridge between a trained reach policy and the policy server interface.

Loads an SB3 PPO model and exposes a `policy(positions, torques) -> action`
function matching what the policy server expects. Uses a lightweight MuJoCo
instance for forward kinematics (joint positions -> TCP position).
"""

import mujoco
import numpy as np
from stable_baselines3 import PPO

from env import MJCF_PATH
from reach.reach_env import RoArmReachEnv


class ReachPolicy:
    """Wraps a trained reach model for use with the policy server."""

    def __init__(self, model_path: str, target: np.ndarray | None = None):
        self.model = PPO.load(model_path)

        # Lightweight MuJoCo model for FK only (no rendering)
        self.mj_model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
        self.mj_data = mujoco.MjData(self.mj_model)

        self._tcp_site_id = mujoco.mj_name2id(
            self.mj_model, mujoco.mjtObj.mjOBJ_SITE, "hand_tcp"
        )
        self._joint_ids = [
            mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in RoArmReachEnv.CONTROLLED_JOINTS
        ]

        # Default target — set via set_target() before calling policy()
        self._target = target if target is not None else np.array([0.15, 0.0, 0.50])

    def set_target(self, target: np.ndarray) -> None:
        self._target = np.array(target, dtype=np.float32)

    def _fk(self, joint_positions: list[float]) -> np.ndarray:
        """Forward kinematics: joint positions -> TCP xyz."""
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        for i, jid in enumerate(self._joint_ids):
            addr = self.mj_model.jnt_qposadr[jid]
            self.mj_data.qpos[addr] = joint_positions[i]
        # Also set hand joint to 0
        hand_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "hand")
        self.mj_data.qpos[self.mj_model.jnt_qposadr[hand_id]] = 0.0
        mujoco.mj_forward(self.mj_model, self.mj_data)
        return self.mj_data.site_xpos[self._tcp_site_id].copy()

    def _build_obs(self, joint_positions: list[float]) -> np.ndarray:
        """Build the 10-dim observation vector from joint positions."""
        joints = np.array(joint_positions[:3], dtype=np.float32)
        tcp = self._fk(joint_positions).astype(np.float32)
        target = self._target.astype(np.float32)
        dist = float(np.linalg.norm(tcp - target))

        # Normalize using the same bounds as RoArmReachEnv
        norm_joints = np.zeros(3, dtype=np.float32)
        for i, name in enumerate(RoArmReachEnv.CONTROLLED_JOINTS):
            lo, hi = RoArmReachEnv.JOINT_LIMITS[name]
            mid = (lo + hi) / 2
            half = (hi - lo) / 2
            norm_joints[i] = (joints[i] - mid) / half

        tcp_mid = (RoArmReachEnv.TCP_LOW + RoArmReachEnv.TCP_HIGH) / 2
        tcp_half = (RoArmReachEnv.TCP_HIGH - RoArmReachEnv.TCP_LOW) / 2
        norm_tcp = ((tcp - tcp_mid) / tcp_half).astype(np.float32)

        tgt_mid = (RoArmReachEnv.TARGET_LOW + RoArmReachEnv.TARGET_HIGH) / 2
        tgt_half = (RoArmReachEnv.TARGET_HIGH - RoArmReachEnv.TARGET_LOW) / 2
        norm_target = ((target - tgt_mid) / tgt_half).astype(np.float32)

        return np.concatenate([norm_joints, norm_tcp, norm_target, [dist / 1.0]])

    def policy(self, positions: list[float], torques: list[float]) -> list[float]:
        """Policy server interface: returns 4 joint position targets.

        Args:
            positions: Current joint positions [base, shoulder, elbow, hand].
            torques: Current joint torques (unused for reach).

        Returns:
            Target joint positions [base, shoulder, elbow, hand].
        """
        obs = self._build_obs(positions)
        action, _ = self.model.predict(obs, deterministic=True)

        # Apply delta (same logic as RoArmReachEnv.step)
        dist = float(np.linalg.norm(self._fk(positions) - self._target))
        t = min(dist / RoArmReachEnv.NEAR_THRESHOLD, 1.0)
        delta_scale = RoArmReachEnv.DELTA_SCALE_NEAR + t * (RoArmReachEnv.DELTA_SCALE_FAR - RoArmReachEnv.DELTA_SCALE_NEAR)
        delta = np.clip(action, -1.0, 1.0) * delta_scale
        current = np.array(positions[:3])
        target_joints = current + delta

        for i, name in enumerate(RoArmReachEnv.CONTROLLED_JOINTS):
            lo, hi = RoArmReachEnv.JOINT_LIMITS[name]
            target_joints[i] = np.clip(target_joints[i], lo, hi)

        return [float(target_joints[0]), float(target_joints[1]), float(target_joints[2]), 0.0]
