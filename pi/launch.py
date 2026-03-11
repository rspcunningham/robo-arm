"""Launch all background ROS2 nodes."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess

VENV_BIN = str(Path(__file__).resolve().parent / ".venv" / "bin")
FOLLOWER_ARM_DEVICE = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "acff0192339bef11b2e7b69061ce3355-if00-port0"
)
LEADER_ARM_DEVICE = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "24ee9dd6f400f0119b83c3295c2a50c9-if00-port0"
)


def generate_launch_description():
    actions = [
        ExecuteProcess(
            cmd=[
                f"{VENV_BIN}/arm",
                "--ros-args",
                "-r",
                "__node:=follower_arm",
                "-p",
                "label:=follower",
                "-p",
                f"serial_device:={FOLLOWER_ARM_DEVICE}",
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                f"{VENV_BIN}/arm",
                "--ros-args",
                "-r",
                "__node:=leader_arm",
                "-p",
                "label:=leader",
                "-p",
                f"serial_device:={LEADER_ARM_DEVICE}",
                "-p",
                "interface_namespace:=/leader",
            ],
            output="screen",
        ),
        ExecuteProcess(cmd=[f"{VENV_BIN}/teleop"], output="screen"),
        ExecuteProcess(cmd=[f"{VENV_BIN}/record-manager"], output="screen"),
        ExecuteProcess(cmd=[f"{VENV_BIN}/cam0"], output="screen"),
    ]
    actions.extend([
        ExecuteProcess(cmd=[f"{VENV_BIN}/control-manager"], output="screen"),
        ExecuteProcess(cmd=[f"{VENV_BIN}/policy-client"], output="screen"),
        ExecuteProcess(cmd=[f"{VENV_BIN}/monitor"], output="screen"),
    ])
    return LaunchDescription(actions)
