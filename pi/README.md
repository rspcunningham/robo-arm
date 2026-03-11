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

`./scripts/run_pi_stack.sh` checks `ROS_DISTRO`, then auto-discovers
`/opt/ros/*/setup.bash`. Runtime defaults for camera and policy are defined
directly in Python node code.

## Run manually on the Pi

```bash
./scripts/run_pi_stack.sh
```

Camera topics:

- primary image topic: `/cam0/image_raw`
- camera calibration/metadata topic: `/cam0/camera_info`
- compressed image topic: `/cam0/image/compressed`
- optional CSI image topic: `/cam2/image_raw`
- optional CSI camera metadata topic: `/cam2/camera_info`
- ESP32 SPI health topic: `/cam1/spi_health`

`cam0` is a USB camera ROS2 node (`nodes/cam0.py`) that defaults to
`/dev/v4l/by-id/usb-MACROSILICON_Guermok_USB2_Video_20250820-video-index0`
(`MJPG`, `1280x720`, `30 fps`) via OpenCV.
It also publishes `/cam0/image/compressed` (JPEG; quality configurable).
`cam2` is the CSI camera ROS2 node (`nodes/cam2.py`) backed by
`python3-libcamera` and expects the CSI camera stack to be enabled on the Pi.
It is available for manual runs (`uv run cam2`) but is not started by default.

## Install as a service on the Pi

```bash
./scripts/install_pi_service.sh
```

That installs the service template from `systemd/robo-arm.service`.
