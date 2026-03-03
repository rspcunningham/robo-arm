# RoArm-M2-S → ROS2 + VLA Learning Platform

## Project Plan

---

## The Big Picture

You're building a 3-layer robotics stack:

```
┌─────────────────────────────────────────────────┐
│  LAYER 3: BRAIN (Mac / Cloud)                   │
│  Inference server (FastAPI/gRPC)                 │
│  SmolVLA, OpenVLA, π0, Octo, etc.               │
│  No ROS2 — just a stateless HTTP endpoint        │
└──────────────────────┬──────────────────────────┘
                       │ HTTP / gRPC (image + joint_state → action chunk)
┌──────────────────────▼──────────────────────────┐
│  LAYER 2: MIDDLEWARE (Raspberry Pi)              │
│  ROS2 Humble, camera, vla_client node,           │
│  roarm_driver, data recording                    │
│  Dev via VS Code Remote-SSH from Mac             │
└──────────────────────┬──────────────────────────┘
                       │ USB Serial (JSON over UART @ 115200)
┌──────────────────────▼──────────────────────────┐
│  LAYER 1: HARDWARE (ESP32 on RoArm-M2-S)        │
│  Servo control, encoder feedback, PID loops      │
│  Stock firmware — receives JSON, returns states   │
└─────────────────────────────────────────────────┘
```

**Key decisions baked in:**

- **ESP32 stays stock.** The existing firmware accepts JSON over UART and returns joint feedback. That's the right abstraction. Don't reflash it.
- **All ROS2 runs on the Pi.** Your Mac never runs ROS2.
- **Mac is a dev machine and inference server.** You write code on Mac via VS Code Remote-SSH, deploy to Pi. Separately, your Mac (or cloud) runs a plain HTTP inference server that the Pi calls.
- **Inference is a network call, not distributed ROS2.** A ROS2 node on the Pi packages up image + joint state, POSTs to your inference endpoint, publishes the returned actions. Swap the URL from `http://macbook.local:8000` to a cloud endpoint with zero code changes on the Pi side.

---

## Hardware You Need

| Item | Purpose | Notes |
|------|---------|-------|
| RoArm-M2-S | The arm | You have this |
| Raspberry Pi 4/5 (4GB+) | ROS2 host, wired to ESP32 | Pi 5 preferred for camera processing |
| USB-C cable | Pi ↔ ESP32 (the middle Type-C port ⑨ on the driver board) | Comes with the arm |
| USB camera | Vision input for VLA | Any USB webcam; consider a wide-angle one mounted on the arm's upper rail |
| 12V 5A power supply | Powers the arm | Included with the arm |
| MicroSD card (32GB+) | Pi OS + ROS2 | |
| Your Mac | Dev machine + inference server | M-series GPU for local model inference via MLX |

---

## Phase 0: Understand What You Already Have (Day 1)

Before changing anything, get familiar with the stock system.

**0.1 — Power on and connect via WiFi**
Connect to the `RoArm-M2` hotspot (password: `12345678`), open `192.168.4.1` in Chrome. Use the web UI to move each joint. Understand the 4 DOF: base (360°), shoulder (180°), elbow (225°), and end effector (gripper 135° or wrist 270°).

**0.2 — Try UART communication from your Mac**
Install the CP2102 driver if needed. Connect the arm to your Mac via USB (middle Type-C port). Find the serial port (`/dev/tty.usbserial-*` on Mac). Run the stock Python serial demo:

```bash
pip install pyserial
python serial_simple_ctrl.py /dev/tty.usbserial-XXXX
```

Send test commands:
- `{"T":104}` — get joint angles feedback
- `{"T":101,"joint":0,"rad":0,"spd":200,"acc":10}` — move base to 0 rad
- `{"T":1041,"x":235,"y":0,"z":234,"t":3.14,"spd":0,"acc":0}` — move to XYZ coordinate

**0.3 — Understand the JSON command set**
The key commands you'll use in your ROS2 driver:

| Command | T-code | Purpose |
|---------|--------|---------|
| Single joint angle control | 121 | Move one joint to a radian angle |
| XYZT direct control | 1041 | Cartesian end-effector control |
| Get joint feedback | 104 | Returns current joint angles |
| Torque on/off | 111 | Enable/disable servo torque |
| DEFA on/off | 112 | Dynamic external force adaptation |
| LED control | 114 | LED on/off |

---

## Phase 1: Raspberry Pi + ROS2 Setup (Days 2–4)

**1.1 — Flash the Pi**
Install Ubuntu 22.04 Server (64-bit) on your Pi. ROS2 Humble targets Ubuntu 22.04.

```bash
# On the Pi
sudo apt update && sudo apt upgrade -y
# Install ROS2 Humble (ros-base is enough — no GUI needed on the Pi)
sudo apt install ros-humble-ros-base
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**1.2 — Set up SSH access**
```bash
# On the Pi
sudo apt install openssh-server
# Note the Pi's IP address
hostname -I
```

From your Mac, verify: `ssh pi@<pi-ip-address>`

**1.3 — Clone and build the Waveshare ROS2 workspace**
Waveshare provides a ROS2 workspace (`roarm_ws_em0` for gripper, `roarm_ws_em1` for wrist):

```bash
cd ~
wget https://files.waveshare.com/wiki/RoArm-M2-S/Roarm_ws_em0.zip
unzip Roarm_ws_em0.zip
cd roarm_ws_em0
# Install dependencies
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

**1.4 — Connect the arm and test**
Plug the arm into the Pi via USB (middle Type-C port ⑨). The ESP32 shows up as `/dev/ttyUSB0`.

```bash
sudo chmod 666 /dev/ttyUSB0
# Launch the driver node
ros2 run roarm_driver roarm_driver
```

In another terminal, verify topics:
```bash
ros2 topic list
ros2 topic echo /joint_states
```

**Milestone: you can see live joint angles updating in `/joint_states`.** Everything from here builds on this.

---

## Phase 2: Dev Workflow — Mac → Pi (Days 3–5, parallel with Phase 1)

**2.1 — VS Code Remote-SSH (the main workflow)**

Install the "Remote - SSH" extension in VS Code. Add your Pi:

1. `Cmd+Shift+P` → "Remote-SSH: Connect to Host" → `ssh pi@<pi-ip>`
2. Open your workspace folder on the Pi (`~/roarm_ws_em0` or wherever your code lives)
3. Install the Python extension on the remote side

Now you're editing files on the Pi with your Mac's editor. The integrated terminal runs on the Pi. `colcon build` runs on the Pi. Everything just works.

**2.2 — SSH config for convenience**
On your Mac, add to `~/.ssh/config`:
```
Host roarm
    HostName <pi-ip-address>
    User pi
    ForwardAgent yes
```
Now `ssh roarm` and VS Code connects to `roarm`.

**2.3 — Git workflow**
Keep your code in a Git repo. The repo lives on GitHub, you clone on Pi, edit via VS Code Remote-SSH, commit/push from Pi terminal. Don't try to sync working directories between Mac and Pi — VS Code Remote-SSH eliminates that need.

**2.4 — Optional: Tailscale for stable networking**
If your Mac and Pi are on the same WiFi, local IPs work. But if you want access from anywhere (coffee shop, different network), install Tailscale on both:

```bash
# On Pi
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# On Mac
brew install tailscale  # or download from tailscale.com
```

Each device gets a stable `100.x.x.x` IP. SSH and inference server calls work regardless of what network you're on.

---

## Phase 3: Build Your Custom ROS2 Nodes (Days 5–10)

The stock Waveshare driver works, but you'll want a cleaner interface for VLA integration.

**3.1 — Create a new ROS2 package**

```bash
cd ~/roarm_ws/src
ros2 pkg create --build-type ament_python roarm_vla \
  --dependencies rclpy sensor_msgs geometry_msgs std_msgs
```

**3.2 — Hardware interface node (roarm_driver)**
If the Waveshare driver does what you need, use it. If you want to customize, write your own that bridges UART JSON ↔ ROS2 topics:

```
Published topics:
  /joint_states          (sensor_msgs/JointState)   — current joint angles at ~20Hz
  /end_effector_pose     (geometry_msgs/Pose)        — FK-computed EE pose

Subscribed topics:
  /joint_commands        (sensor_msgs/JointState)   — target joint angles
  /cartesian_commands    (geometry_msgs/Pose)        — target XYZ + gripper

Services:
  /torque_enable         (std_srvs/SetBool)
  /go_home               (std_srvs/Trigger)
```

**3.3 — Camera node**
```bash
sudo apt install ros-humble-usb-cam
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```
This publishes `/image_raw` as `sensor_msgs/Image`. This is what gets sent to the inference server.

**3.4 — VLA client node (the key integration piece)**
This runs on the Pi, calls your inference server over HTTP:

```python
# Pseudocode for the vla_client_node
class VLAClientNode(Node):
    def __init__(self):
        # Subscribe to camera and joint states
        self.image_sub = self.create_subscription(Image, '/image_raw', ...)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', ...)
        # Publish actions
        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', ...)
        # Inference server URL — configurable via ROS2 param
        self.declare_parameter('inference_url', 'http://macbook.local:8000/predict')
        # Control loop timer
        self.create_timer(0.1, self.control_loop)  # 10Hz

    def control_loop(self):
        # Package latest image + joint state
        payload = {
            "image": base64_encode(self.latest_image),
            "joint_state": self.latest_joints,
            "instruction": self.current_task  # e.g. "pick up the red block"
        }
        # Call inference server
        response = requests.post(self.inference_url, json=payload)
        action_chunk = response.json()["actions"]
        # Publish first action (or interpolate through the chunk)
        self.cmd_pub.publish(action_chunk[0])
```

The `inference_url` parameter is the key abstraction. Set it to:
- `http://macbook.local:8000/predict` — Mac on local network
- `http://100.x.x.x:8000/predict` — Mac via Tailscale
- `https://your-cloud-endpoint.com/predict` — cloud inference
- `http://localhost:8000/predict` — if you ever run inference on the Pi itself

**3.5 — Data recording node**
For collecting demonstration data (critical for VLA fine-tuning):

```
Subscribes to:
  /joint_states
  /image_raw

Records to:
  HDF5 or LeRobot dataset format

Modes:
  - Teleoperation recording (torque off, manually guide arm, record trajectory + images)
  - Replay (play back recorded trajectories)
```

The Waveshare "HORIZONTAL DRAG" / "VERTICAL DRAG" modes are useful here — they let you manually guide the arm while it reports joint states. Turn torque off with `{"T":111,"cmd":0}`, move the arm by hand, record everything.

---

## Phase 4: Inference Server on Mac (Days 8–12)

This runs on your Mac. It is a plain Python web server. It knows nothing about ROS2.

**4.1 — Basic skeleton**

```python
# inference_server.py — runs on your Mac
from fastapi import FastAPI
import uvicorn

app = FastAPI()

@app.post("/predict")
async def predict(request: dict):
    image = decode_image(request["image"])
    joint_state = request["joint_state"]
    instruction = request.get("instruction", "")

    # Run your model
    actions = model.predict(image, joint_state, instruction)

    return {"actions": actions.tolist()}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

**4.2 — Start with a dummy policy**
Before touching any ML, return random or scripted actions. This lets you test the full pipeline end-to-end: camera → Pi → HTTP → Mac → response → Pi → arm moves.

**4.3 — Plug in real models progressively**

| Stage | Model | Size | Runs on | What you learn |
|-------|-------|------|---------|----------------|
| Dummy | Scripted | — | Mac CPU | End-to-end pipeline works |
| Classical | OpenCV + IK | — | Mac or Pi CPU | Vision + control basics |
| First VLA | SmolVLA | 450M | Mac GPU (MLX/MPS) | VLA inference, action chunks, LeRobot |
| Lightweight | Octo | 93M | Mac GPU or even Pi | Diffusion policies, generalist models |
| Full VLA | OpenVLA | 7B | Cloud GPU | Fine-tuning, larger models, quantization |
| SOTA | π0 | ~3B | Cloud GPU | Flow matching, high-frequency control |

**4.4 — LeRobot integration**
Hugging Face's LeRobot is the easiest on-ramp to VLA inference and training:

```bash
# On your Mac
pip install lerobot
```

LeRobot provides pretrained SmolVLA, data collection tools, and training scripts. Your inference server wraps LeRobot's prediction API in an HTTP endpoint.

---

## Phase 5: Collect Data and Fine-Tune (Days 15–30)

This is where real learning happens.

**5.1 — Teleoperation for data collection**
1. Turn off torque: `{"T":111,"cmd":0}`
2. Start the recording node
3. Physically guide the arm through a task while camera records
4. Repeat 50–100 times for a single task

**5.2 — Convert to LeRobot dataset format**
LeRobot expects HDF5 with images, joint states, and actions per timestep. Write a converter from your recorded ROS2 bags or HDF5 files.

**5.3 — Fine-tune SmolVLA or Octo**
- **On your Mac**: possible for SmolVLA with MLX, but slow
- **On cloud GPU**: Lambda Labs, RunPod, or vast.ai — rent an A100 for a few hours
- **On Hugging Face**: AutoTrain or Spaces for managed fine-tuning

**5.4 — Deploy and iterate**
Update your inference server with the fine-tuned model. Test on real arm. Collect failure cases. Fine-tune again. This loop is the core of applied robot learning.

---

## Phase 6: Advanced Topics (Ongoing)

**Simulation**
- Set up the URDF in Gazebo or Isaac Sim
- Train policies in sim, transfer to real arm (sim-to-real)
- The Waveshare ROS2 package already includes URDF files in `roarm_description`

**Diffusion policies**
- Implement a diffusion policy head instead of direct action regression
- This is what π0, Octo, and most modern VLAs use
- Chi et al.'s "Diffusion Policy" paper is the canonical reference

**Teleoperation improvements**
- Build a leader-follower setup using ESP-NOW (the arm supports this natively with a second arm)
- Or use a gamepad via ROS2 (`ros2 launch roarm_moveit_servo servo_control.launch.py`)
- Better teleop → better data → better VLA performance

**Multi-camera setup**
- Wrist camera + overhead camera (most VLAs benefit from multiple viewpoints)
- Mount a camera on the arm's upper rail using the M4 boat nut slots

**Dual-system architecture**
Run a large VLM (GPT-4o, Claude, Gemini) for high-level task planning, combined with a local model for low-level control. This mirrors NVIDIA GR00T N1 and Figure AI Helix at hobby scale:

```
Cloud VLM: "To pick up the red cup, first approach from the left,
            open gripper, move to coordinates near the cup handle..."
    ↓ subtask sequence
Local policy (SmolVLA/Octo on Mac): generates 10Hz action chunks
    ↓ joint commands
Pi → arm executes
```

---

## Key Resources

### RoArm-M2-S Specific
- [Waveshare Wiki (main page)](https://www.waveshare.com/wiki/RoArm-M2-S)
- [JSON Command Reference](https://www.waveshare.com/wiki/RoArm-M2-S_JSON_Command_Meaning)
- [GitHub: ESP32 firmware](https://github.com/waveshareteam/roarm_m2)
- [GitHub: ROS2 workspace](https://github.com/waveshareteam/roarm_ws)
- [GitHub: roarm_ws_em0 (gripper ROS2)](https://github.com/waveshareteam/roarm_ws_em0)
- [GitHub: RoArm Python SDK](https://github.com/waveshareteam/waveshare_roarm_sdk)
- [ROS2 + MoveIt2 Tutorial](https://www.waveshare.com/wiki/RoArm-M2-S_ROS2_Humble_%2B_Moveit2_Tutorial)
- [Python UART Communication](https://www.waveshare.com/wiki/RoArm-M2-S_Python_UART_Communication)

### ROS2 Learning
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- The Construct — free ROS2 courses online

### VLA / Robot Learning
- [LeRobot by Hugging Face](https://github.com/huggingface/lerobot) — most accessible VLA framework
- [SmolVLA](https://huggingface.co/lerobot/smolvla) — best starting VLA for this hardware (450M params)
- [Octo](https://octo-models.github.io/) — lightweight generalist policy (93M params)
- [OpenVLA](https://openvla.github.io/) — open-source 7B VLA
- [π0 paper](https://www.pi.website/download/pi0.pdf) — state-of-the-art flow-matching VLA
- [VLA Survey](https://vla-survey.github.io/) — comprehensive field overview
- [Diffusion Policy](https://diffusion-policy.cs.columbia.edu/) — foundational work on diffusion for robot control

---

## Suggested Timeline

| Week | Focus | Milestone |
|------|-------|-----------|
| 1 | Phase 0 + 1 + 2 | Arm on Pi, ROS2 running, VS Code Remote-SSH working |
| 2 | Phase 3 | Custom ROS2 nodes, camera publishing, VLA client node calling dummy server |
| 2–3 | Phase 4 (dummy → classical) | Full pipeline working end-to-end, simple OpenCV pick-and-place |
| 3–4 | Phase 4 (first VLA) | SmolVLA running on Mac, controlling arm over HTTP |
| 4–6 | Phase 5 | Collected demo data, fine-tuned a model, tested on real arm |
| 6+ | Phase 6 | Diffusion policies, dual-system architecture, sim-to-real |

---

## Why Not Reflash the ESP32

The ESP32's existing firmware handles PID control of the ST3215 bus servos, manages encoder feedback at the hardware level, and exposes a clean JSON-over-UART API. Rewriting this would mean reimplementing servo bus protocol handling, losing the useful web UI and ESP-NOW teleoperation capability, and spending months on firmware work with no learning benefit.

The wired USB connection between Pi and ESP32 gives ~1-5ms round-trip latency, more than sufficient for the 10-50Hz control loops that VLA models operate at. This is the standard pattern in ROS2 robotics: hardware controllers stay "dumb" and fast, intelligence runs on the host.
