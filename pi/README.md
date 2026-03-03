# Pi Runtime

This is the Raspberry Pi project for the robot runtime.

It contains:

- the ROS2-facing nodes in `nodes/`
- the ESP32 camera tooling in `esp.py` and `esp32-camera/`
- the Pi service/runtime scripts in `scripts/`

## Install on the Pi

```bash
cd pi
uv sync
```

`./scripts/run_pi_stack.sh` uses `ROS_SETUP_BASH` if set. Otherwise it checks `ROS_DISTRO`, then auto-discovers `/opt/ros/*/setup.bash`.

## Run manually on the Pi

```bash
./scripts/run_pi_stack.sh
```

Set `DISABLE_CAM=1` to start without the camera node.

## Install as a service on the Pi

```bash
./scripts/install_pi_service.sh
```

That installs the service template from `systemd/robo-arm.service`.
