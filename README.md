# Robo Arm

This repo now contains sibling projects:

- `pi/`: the Raspberry Pi runtime project
- `policy_server/`: the Mac/cloud policy inference service
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

Set `ROBO_ARM_POLICY_URL` before starting the Pi runtime when the policy server is
running on another machine:

```bash
cd pi
ROBO_ARM_POLICY_URL=http://macbook.local:8000/predict ./scripts/run_pi_stack.sh
```

## Policy server

Run the policy service on your Mac (or any reachable host):

```bash
cd policy_server
uv sync
uv run serve-policy policy_demo.py:policy --task "pick up the coffee cup" --host 0.0.0.0 --port 8000
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

This syncs only `./pi` to the Pi, runs `uv sync --project pi --frozen`, and restarts `robo-arm.service` if installed.

Set `ROBO_ARM_POLICY_URL` inline on the Mac before deploy when the Pi should
call a policy server running somewhere else:

```bash
ROBO_ARM_POLICY_URL=http://Robins-MacBook-Pro.local:8000/predict ./scripts/deploy_pi.sh
```

Defaults:

- `PI_HOST=pi@pi1.local`
- `PI_DIR=/home/pi/robo-arm`

Override them inline when needed:

```bash
PI_HOST=pi@192.168.1.50 PI_DIR=/home/pi/code ./scripts/deploy_pi.sh
```

## Run transient Pi tasks

Run one-off Pi-side ROS 2 tools from the Mac:

```bash
./scripts/record_pi.sh --repo-id my-org/demos --fps 10 --task "pick up block" --num-episodes 1
./scripts/replay_pi.sh --repo-id my-org/demos --episode 0
```

These SSH into the Pi, source ROS 2, enter `pi/`, and run the installed `record` or `replay` entrypoint against the always-on `robo-arm.service` runtime.

## Notes

- The layout and workflow are described in [docs/mac-pi-dev-workflow.md](/Users/robin/Desktop/robo-arm/docs/mac-pi-dev-workflow.md).
- The older plan in [docs/roarm-m2s-ros2-vla-project-plan.md](/Users/robin/Desktop/robo-arm/docs/roarm-m2s-ros2-vla-project-plan.md) still reflects the pre-split layout.
