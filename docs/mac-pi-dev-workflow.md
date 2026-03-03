# Mac Dev, Pi Deploy

## Goal

Keep one repo, develop from the Mac, deploy the hardware-facing runtime to the Raspberry Pi, and avoid installing Mac-only ML or training dependencies on the Pi.

## Project layout

The repo now uses sibling projects:

- [pi/](/Users/robin/Desktop/robo-arm/pi): Raspberry Pi runtime project
- [experiments/](/Users/robin/Desktop/robo-arm/experiments): Mac-only experiments project

The Pi project contains the existing robot runtime:

- [pi/nodes/arm.py](/Users/robin/Desktop/robo-arm/pi/nodes/arm.py): serial driver for the arm
- [pi/nodes/cam.py](/Users/robin/Desktop/robo-arm/pi/nodes/cam.py): SPI camera bridge
- [pi/nodes/monitor.py](/Users/robin/Desktop/robo-arm/pi/nodes/monitor.py): browser monitor
- [pi/nodes/record.py](/Users/robin/Desktop/robo-arm/pi/nodes/record.py): dataset recording
- [pi/nodes/replay.py](/Users/robin/Desktop/robo-arm/pi/nodes/replay.py): dataset replay

Use [pi/scripts/run_pi_stack.sh](/Users/robin/Desktop/robo-arm/pi/scripts/run_pi_stack.sh) as the supported Pi runtime entrypoint and [scripts/deploy_pi.sh](/Users/robin/Desktop/robo-arm/scripts/deploy_pi.sh) for the Mac-to-Pi inner loop.

The Pi runtime script supports an explicit `ROS_SETUP_BASH` override. Otherwise it checks `ROS_DISTRO`, then auto-discovers `/opt/ros/*/setup.bash`.

## Install rules

The boundary is now simple:

- `pi/` contains dependencies the Pi needs
- `experiments/` contains dependencies only your Mac needs

Examples:

```bash
cd pi
uv sync

cd experiments
uv sync
```

If you are adding a dependency:

- add it in `pi/` only if the Pi must have it
- otherwise add it in `experiments/`

## Inner loop

Use this as the default workflow:

```text
edit on Mac -> ./scripts/deploy_pi.sh -> Pi service restarts -> test on hardware
```

The deploy script syncs the repo root and runs `uv sync --project pi --frozen` on the Pi.

Commit and push only after the Pi-side behavior is correct.
