# MuJoCo Simulation for RoArm-M2-S

## Files

| File | Purpose |
|------|---------|
| `roarm.xml` | The MuJoCo model — arm + scene (table, cameras, actuators). This is the "world" the sim runs in. |
| `env.py` | Python interface to the sim. `RoArmSimEnv` with `reset()`, `step()`, `observe()`, `render()`. Import this in training scripts. |
| `demo.py` | Visual debugging — open the viewer, sweep joints, save camera images. |
| `policy_loop.py` | Connects the sim to the policy server over HTTP. For testing a trained policy in sim before deploying to the real arm. |
| `roarm_description.urdf` | Original Waveshare URDF (reference). |
| `roarm_converted.xml` | Raw MuJoCo conversion of the URDF (reference). |
| `meshes/` | STL mesh files from the official Waveshare URDF. |

## Setup

```bash
cd sim
uv sync
```

## Running

```bash
# Interactive viewer (inspect the arm, rotate camera, etc.)
uv run python demo.py

# Joint range sweep — watch each joint move through its limits
# (needs mjpython on macOS for programmatic viewer control)
uv run mjpython demo.py --test-range

# Render camera view to file
uv run python demo.py --save-image test.png

# Policy loop — connect sim to policy server
# (start policy server first: cd ../policy_server && uv run policy-server)
uv run python policy_loop.py              # headless
uv run mjpython policy_loop.py --viewer   # with 3D viewer
```

### macOS note

On macOS, `python` works for the plain interactive viewer and headless mode. Anything that uses programmatic viewer control (`--test-range`, `--viewer`) requires `mjpython` — this is a MuJoCo requirement, not something specific to this project. `mjpython` is installed automatically with the mujoco package.

## How the model was built

The MJCF model (`roarm.xml`) is based on the official Waveshare URDF from `waveshareteam/roarm_ws_em0` (branch `ros2-humble`). The process:

1. **URDF mesh paths** were changed from `package://roarm_description/meshes/` to `meshes/` to work locally
2. **Converted to MJCF** using MuJoCo's built-in converter:
   ```python
   model = mujoco.MjModel.from_xml_path("roarm_description.urdf")
   mujoco.mj_saveLastXML("roarm_converted.xml", model)
   ```
   This handles the kinematic chain, joint axes/limits, masses, inertias (including full tensor diagonalization), and mesh references.
3. **Added scene elements** that URDF doesn't support: ground plane, table, cameras, lighting, target block
4. **Added actuators** — position controllers on each joint to match the real servo behavior
5. **Added physics settings** — `implicitfast` integrator (handles stiff position actuators without instability), joint damping, armature
6. **Disabled self-collision** between arm links — URDF simulators exclude adjacent links automatically, MuJoCo doesn't. Without this, overlapping meshes at joint origins lock the joints.

## What's accurate (from URDF)

- Kinematic chain, joint axes, joint limits
- Masses and inertias (CAD-derived, full tensor)
- STL mesh geometry

## What needs tuning against the real arm

These are reasonable defaults but not calibrated to hardware:

- **Actuator gains (`kp`, `kv`)** — How stiff/responsive the sim joints are. To calibrate: command the real arm to a position, record the trajectory (acceleration, overshoot, settling time), tune sim gains to match.
- **Joint damping & armature** — Affects oscillation and energy dissipation. Same calibration approach.
- **Camera position** — `esp_cam` is an approximation of the ESP32-CAM viewpoint. To calibrate: compare a real camera frame to a sim render and adjust.

These matter most for sim-to-real transfer of learned policies. For basic testing and RL training, the defaults are fine.

## Using env.py in training scripts

```python
from env import RoArmSimEnv

env = RoArmSimEnv()
env.reset()                           # reset to zero position
env.reset(qpos=[0, -0.5, 1.0, 0.5])  # reset to specific joint angles

env.step([0.0, -0.5, 1.0, 0.5])      # command joint positions, advance physics
positions = env.get_joint_positions()  # [base, shoulder, elbow, hand]
tcp = env.get_tcp_position()           # [x, y, z] end-effector position
rgb = env.render()                     # numpy array (480, 640, 3)
obs = env.observe()                    # {"image_jpeg_b64": ..., "joint_state": ...}

env.close()
```

Joint names: `base` (rotate), `shoulder` (lift), `elbow` (extend), `hand` (gripper).
Joint angles are in radians, same convention as the real arm's ROS topics.
