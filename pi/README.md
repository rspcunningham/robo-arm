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

Install the libcamera Python bindings once on the Pi:

```bash
sudo apt install -y python3-libcamera
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

Camera topics:

- primary image topic: `/cam0/image_raw`
- camera calibration/metadata topic: `/cam0/camera_info`
- ESP32 SPI health topic: `/cam1/spi_health`

`cam0` is a native Python ROS2 node (`nodes/cam0.py`) backed by
`python3-libcamera` and expects the CSI camera stack to be enabled on the Pi.

## Install as a service on the Pi

```bash
./scripts/install_pi_service.sh
```

That installs the service template from `systemd/robo-arm.service`.
