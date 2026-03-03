# Robo Arm

This repo now contains sibling projects:

- `pi/`: the Raspberry Pi runtime project
- `experiments/`: the Mac-only project for local experiments

Use the repo root as the shared container for source control, docs, and deploy helpers.

## Pi runtime

The current robot runtime lives in [pi/](/Users/robin/Desktop/robo-arm/pi).

Install or update it on the Pi:

```bash
cd pi
uv sync
```

The runtime script will use `ROS_SETUP_BASH` if you set it. Otherwise it checks `ROS_DISTRO`, then auto-discovers `/opt/ros/*/setup.bash`.

Run it manually on the Pi:

```bash
cd pi
./scripts/run_pi_stack.sh
```

Install it as a service on the Pi:

```bash
cd pi
./scripts/install_pi_service.sh
```

## Mac experiments

Use [experiments/](/Users/robin/Desktop/robo-arm/experiments) for Mac-only dependencies and scripts.

```bash
cd experiments
uv sync
```

If a dependency should never land on the Pi, add it in `experiments/`, not in `pi/`.

## Deploy from the Mac

From the repo root:

```bash
./scripts/deploy_pi.sh
```

This syncs the repo to the Pi, runs `uv sync --project pi --frozen`, and restarts `robo-arm.service` if installed.

Defaults:

- `PI_HOST=pi@pi1.local`
- `PI_DIR=/home/pi/robo-arm`

Override them inline when needed:

```bash
PI_HOST=pi@192.168.1.50 PI_DIR=/home/pi/code ./scripts/deploy_pi.sh
```

## Notes

- The layout and workflow are described in [docs/mac-pi-dev-workflow.md](/Users/robin/Desktop/robo-arm/docs/mac-pi-dev-workflow.md).
- The older plan in [docs/roarm-m2s-ros2-vla-project-plan.md](/Users/robin/Desktop/robo-arm/docs/roarm-m2s-ros2-vla-project-plan.md) still reflects the pre-split layout.
