# Quickstart

1. Install Ubuntu Server 24.04 to an SD card using the Raspberry Pi imager. Ensure you give yourself ssh access and an internet connection. 
2. Boot up the pi. 
3. SSH in to the pi and run the below commands one at a time:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl git htop
sudo apt install -y gh
gh auth login # and follow the prompts to sign in
curl -LsSf https://astral.sh/uv/install.sh | sh
curl -fsSL https://cli.anthropic.com/install.sh | sh
source ~/.bashrc
```

Then, run `claude` in the root dir to launch claude code and sign in

For esp32 camera use, run the following prompt: 

```markdown
You are running on a raspberry pi running ubuntu server 24

Tasks:
1. use the gh cli to clone rspcunningham/robo-arm to a dir named 'code'
2. set up ros2
3. enable UART over the raspberry pi's gpio pins
4. verify that the esp compiling/flashing toolchain in the cloned source works with the 'uv run esp check' cli command
5. flash the 'esp-camera' code to the esp32 connected over uart
6. check that the robot arm is available as a usb serial device, reference './docs' in the cloned repo
7. ensure the camera and arm work as ros2 nodes with 'uv run launch-nodes' in the cloned repo

ultimate goal is vla operation. 
```

If using without the camera, run: 

```'markdown
You are running on a raspberry pi running ubuntu server 24

Tasks:
1. use the gh cli to clone rspcunningham/robo-arm to a dir named 'code'
2. set up ros2
3. check that the robot arm is available as a usb serial device, reference './docs' in the cloned repo
4. ensure the arm works as a ros2 nodes with 'uv run launch-nodes' in the cloned repo -- note that this expects the cam node to be available, but it isnt. I'm not sure how to handle that. 

ultimate goal is vla operation. 
'''
