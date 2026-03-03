# RoArm-M2-S ROS2 Reference

Sources:

- <https://www.waveshare.com/wiki/RoArm-M2-S_2._ROS2_Workspace_Description>
- <https://www.waveshare.com/wiki/RoArm-M2-S_3._Drive_Nodes_to_Control_Real-world_Robotic_Arm>
- <https://www.waveshare.com/wiki/RoArm-M2-S_8._Control_Robotic_Arm_with_Commands>
- <https://github.com/waveshareteam/roarm_ws_em0>

Fetched: 2026-03-02  
Conversion note: condensed from Waveshare wiki `action=raw` pages plus direct inspection of the upstream `roarm_ws_em0` repo.

## Workspace Layout

The ROS2 workspace is `roarm_ws_em0`.

Main packages:

- `roarm_driver`: talks to the physical arm over serial
- `roarm_description`: URDF and visualization assets
- `roarm_moveit`: MoveIt configuration and services
- `roarm_moveit_ikfast_plugins`: IKFast kinematics plugin
- `roarm_moveit_cmd`: service and command nodes
- `roarm_web_app`: web control
- `moveit_servo`: gamepad / servo integration

The upstream repo we inspected matches that layout under `src/roarm_main/`.

## Driver Node Workflow

From the Waveshare docs:

1. Identify the serial port after plugging into the middle USB-C port.
2. On Ubuntu, the default is expected to be `/dev/ttyUSB0`.
3. Grant serial access:

```bash
sudo chmod 666 /dev/ttyUSB0
```

4. Build the workspace:

```bash
cd ~/roarm_ws_em0
colcon build
source install/setup.bash
```

5. Run the driver:

```bash
ros2 run roarm_driver roarm_driver
```

## What `roarm_driver` Actually Does

From direct inspection of the upstream `waveshareteam/roarm_ws_em0` file `src/roarm_main/roarm_driver/roarm_driver/roarm_driver.py`:

- Declares parameters:
  - `serial_port` (default `/dev/ttyUSB0`)
  - `baud_rate` (default `115200`)
- Opens the serial port directly.
- Immediately sends:

```json
{"T":605,"cmd":0}
```

That disables serial debug printing from the firmware so the line-oriented JSON stream is cleaner.

### Subscriptions

The node subscribes to:

- `joint_states` (`sensor_msgs/msg/JointState`)
- `hand_pose` (`geometry_msgs/msg/Pose`)
- `led_ctrl` (`std_msgs/msg/Float32`)

### JointState to Firmware Mapping

In `joint_states_callback`, the driver converts ROS joint names to a `T:102` all-joints-radians command:

```json
{"T":102,"base":...,"shoulder":...,"elbow":...,"hand":...,"spd":0,"acc":10}
```

The mapping currently used:

- `base = -position['base_link_to_link1']`
- `shoulder = -position['link1_to_link2']`
- `elbow = position['link2_to_link3']`
- `hand = 3.1415926 - position['link3_to_gripper_link']`

That sign inversion is important if you write a replacement driver.

### Pose Handling

In `pose_callback`, the driver sends:

```json
{"T":105}
```

and expects a `T:1051` feedback object.

However, the current implementation is not clean:

- it creates a new `BaseController` inside the callback
- that opens a second serial connection each time
- it reads feedback opportunistically rather than owning a single reader thread

For a custom ROS2 driver, you should not copy that pattern. Use one persistent serial connection and one parser.

### LED Control

`led_ctrl` is forwarded directly as:

```json
{"T":114,"led":<float32-data>}
```

## MoveIt Command Layer

From the wiki and repo:

- `roarm_moveit_cmd` exposes nodes and services on top of MoveIt planning.
- These nodes are not the firmware protocol; they are higher-level ROS interfaces.

### Start Command-Control Stack

```bash
cd ~/roarm_ws_em0
ros2 launch roarm_moveit_cmd command_control.launch.py
```

### Get Current Pose

Node:

```bash
ros2 run roarm_moveit_cmd getposecmd
```

Service:

```bash
ros2 service call /get_pose_cmd roarm_moveit/srv/GetPoseCmd
```

### Move End-Effector to a Point

Node:

```bash
ros2 run roarm_moveit_cmd movepointcmd
```

Service:

```bash
ros2 service call /move_point_cmd roarm_moveit/srv/MovePointCmd "{x: 0.2, y: 0, z: 0}"
```

`MovePointCmd.srv` is:

```srv
float64 x
float64 y
float64 z
---
bool success
string message
```

The repo implementation converts meters to millimeters for IK and flips the sign of `y` before planning.

### Control the Gripper

Node:

```bash
ros2 run roarm_moveit_cmd setgrippercmd
```

Topic example from the wiki:

```bash
ros2 topic pub /gripper_cmd /gripper_cmd "{data: 0.2}"
```

This is described as a gripper angle in radians.

### Draw a Circle

Node:

```bash
ros2 run roarm_moveit_cmd movecirclecmd
```

Service:

```bash
ros2 service call /move_circle_cmd roarm_moveit/srv/MoveCircleCmd "{x: 0.2, y: 0, z: 0, radius: 0.1}"
```

## Implications For A Custom ROS2 Node

If the goal is a stable VLA-facing driver, the clean architecture is:

- one serial transport object
- one newline-delimited JSON parser
- one request/response path for `T:105` feedback
- one command publisher path for `T:102`, `T:1041`, `T:114`, and safety commands
- explicit translation between ROS coordinate conventions and the firmware's sign conventions

The stock `roarm_driver` proves the protocol works, but it is not a good long-term design reference for serial ownership, error handling, or state synchronization.
