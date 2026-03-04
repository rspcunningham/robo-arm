# Pi Runtime

This is the Raspberry Pi project for the robot runtime.

It contains:

- the ROS2-facing nodes in `nodes/`
- the remote policy client in `nodes/policy_client.py`
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

Point the always-on `policy-client` at your Mac policy server with
`ROBO_ARM_POLICY_URL` when needed:

```bash
ROBO_ARM_POLICY_URL=http://macbook.local:8000/predict ./scripts/run_pi_stack.sh
```

## Install as a service on the Pi

```bash
./scripts/install_pi_service.sh
```

That installs the service template from `systemd/robo-arm.service`.
