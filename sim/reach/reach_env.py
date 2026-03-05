"""Gymnasium environment for the reach task on the RoArm-M2-S."""

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from env import RoArmSimEnv


class RoArmReachEnv(gym.Env):
    """Move the TCP to a random target position.

    Action:  delta joint positions for base, shoulder, elbow (3,).
    Obs:     joint positions (3) + TCP xyz (3) + target xyz (3) + distance (1) = 10.
    Reward:  -distance + 10.0 bonus on success (TCP within 2 cm).
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 20}

    # Joint limits (from roarm.xml)
    JOINT_LIMITS = {
        "base": (-3.1416, 3.1416),
        "shoulder": (-1.5708, 1.5708),
        "elbow": (-1.0, 3.1416),
    }
    CONTROLLED_JOINTS = ["base", "shoulder", "elbow"]

    # Normalization bounds (from workspace sampling — covers full reachable volume)
    TCP_LOW = np.array([-0.55, -0.55, 0.15], dtype=np.float32)
    TCP_HIGH = np.array([0.55, 0.55, 1.05], dtype=np.float32)
    # Target sampling in cylindrical coords around arm base (0, 0)
    TARGET_RADIUS_MIN = 0.10
    TARGET_RADIUS_MAX = 0.30
    TARGET_Z_MIN = 0.40
    TARGET_Z_MAX = 0.70

    SUCCESS_THRESHOLD = 0.03  # 3 cm
    MAX_STEPS = 200
    DELTA_SCALE_FAR = 0.15   # max joint delta when far from target
    DELTA_SCALE_NEAR = 0.03  # max joint delta when close to target
    NEAR_THRESHOLD = 0.08    # distance at which we switch to fine control

    def __init__(self, render_mode: str | None = None):
        super().__init__()
        self.render_mode = render_mode
        self.sim = RoArmSimEnv()

        # Target block freejoint qpos address (7 DoF: 3 pos + 4 quat)
        self._target_jnt_id = mujoco.mj_name2id(
            self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, "target_joint"
        )
        self._target_qpos_addr = self.sim.model.jnt_qposadr[self._target_jnt_id]
        self._target_qvel_addr = self.sim.model.jnt_dofadr[self._target_jnt_id]

        # Controlled joint indices (base, shoulder, elbow — skip hand)
        self._ctrl_joint_ids = [
            mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in self.CONTROLLED_JOINTS
        ]
        self._ctrl_actuator_ids = [
            mujoco.mj_name2id(
                self.sim.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{n}_actuator"
            )
            for n in self.CONTROLLED_JOINTS
        ]

        # Spaces
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32
        )

        self._target_pos = np.zeros(3, dtype=np.float32)
        self._prev_dist = 0.0
        self._step_count = 0

    def _normalize_joints(self, joints: np.ndarray) -> np.ndarray:
        """Normalize joint positions to ~[-1, 1] using known limits."""
        result = np.zeros(3, dtype=np.float32)
        for i, name in enumerate(self.CONTROLLED_JOINTS):
            lo, hi = self.JOINT_LIMITS[name]
            mid = (lo + hi) / 2
            half = (hi - lo) / 2
            result[i] = (joints[i] - mid) / half
        return result

    def _normalize_pos(self, pos: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
        """Normalize a 3D position to ~[-1, 1]."""
        mid = (low + high) / 2
        half = (high - low) / 2
        return ((pos - mid) / half).astype(np.float32)

    def _get_obs(self) -> np.ndarray:
        joints = np.array([
            float(self.sim.data.qpos[self.sim.model.jnt_qposadr[jid]])
            for jid in self._ctrl_joint_ids
        ], dtype=np.float32)

        tcp = self.sim.get_tcp_position().astype(np.float32)
        dist = float(np.linalg.norm(tcp - self._target_pos))

        obs = np.concatenate([
            self._normalize_joints(joints),
            self._normalize_pos(tcp, self.TCP_LOW, self.TCP_HIGH),
            self._normalize_pos(self._target_pos, self.TCP_LOW, self.TCP_HIGH),
            [dist / 1.0],  # normalize distance (max ~1.0m)
        ])
        return obs.astype(np.float32)

    def _randomize_target(self, rng: np.random.Generator) -> None:
        """Place target at a random reachable position in a ring around the arm."""
        angle = rng.uniform(0, 2 * np.pi)
        radius = rng.uniform(self.TARGET_RADIUS_MIN, self.TARGET_RADIUS_MAX)
        x = float(radius * np.cos(angle))
        y = float(radius * np.sin(angle))
        z = float(rng.uniform(self.TARGET_Z_MIN, self.TARGET_Z_MAX))
        self._target_pos = np.array([x, y, z], dtype=np.float32)
        # Set freejoint position (qpos[addr:addr+3] = xyz, addr+3:addr+7 = quat)
        addr = self._target_qpos_addr
        self.sim.data.qpos[addr: addr + 3] = self._target_pos
        self.sim.data.qpos[addr + 3: addr + 7] = [1, 0, 0, 0]  # identity quat

    def _pin_target(self) -> None:
        """Hold the target block at the intended position (override gravity)."""
        # Reset position (gravity moves it during substeps)
        addr = self._target_qpos_addr
        self.sim.data.qpos[addr: addr + 3] = self._target_pos
        self.sim.data.qpos[addr + 3: addr + 7] = [1, 0, 0, 0]
        # Zero velocity
        vaddr = self._target_qvel_addr
        self.sim.data.qvel[vaddr: vaddr + 6] = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random

        # Reset sim to home position (all joints at 0)
        self.sim.reset(qpos=[0.0, 0.0, 0.0, 0.0])

        # Randomize target
        self._randomize_target(rng)
        self._pin_target()
        mujoco.mj_forward(self.sim.model, self.sim.data)

        self._step_count = 0
        tcp = self.sim.get_tcp_position()
        self._prev_dist = float(np.linalg.norm(tcp - self._target_pos))
        return self._get_obs(), {}

    def step(self, action):
        self._step_count += 1

        # Scale action — large steps when far, fine steps when close
        t = min(self._prev_dist / self.NEAR_THRESHOLD, 1.0)  # 0 when at target, 1 when far
        delta_scale = self.DELTA_SCALE_NEAR + t * (self.DELTA_SCALE_FAR - self.DELTA_SCALE_NEAR)
        delta = np.clip(action, -1.0, 1.0) * delta_scale

        # Current joint positions for controlled joints
        current = np.array([
            float(self.sim.data.qpos[self.sim.model.jnt_qposadr[jid]])
            for jid in self._ctrl_joint_ids
        ])

        # Apply delta and clip to joint limits
        target_joints = current + delta
        for i, name in enumerate(self.CONTROLLED_JOINTS):
            lo, hi = self.JOINT_LIMITS[name]
            target_joints[i] = np.clip(target_joints[i], lo, hi)

        # Full 4-DOF action (hand fixed at 0)
        full_action = [target_joints[0], target_joints[1], target_joints[2], 0.0]
        self.sim.step(full_action)

        # Pin target in place
        self._pin_target()

        # Compute reward
        tcp = self.sim.get_tcp_position().astype(np.float32)
        dist = float(np.linalg.norm(tcp - self._target_pos))

        # Reward for getting closer + bonus on contact
        reward = (self._prev_dist - dist) * 10.0 - dist * 0.1
        self._prev_dist = dist
        terminated = False
        if dist < self.SUCCESS_THRESHOLD:
            reward += 20.0
            terminated = True
        truncated = self._step_count >= self.MAX_STEPS

        obs = self._get_obs()
        info = {"distance": dist, "success": dist < self.SUCCESS_THRESHOLD}

        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "rgb_array":
            return self.sim.render()
        return None

    def close(self):
        self.sim.close()
