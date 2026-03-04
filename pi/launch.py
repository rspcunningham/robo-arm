"""Launch all background ROS2 nodes."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess

VENV_BIN = str(Path(__file__).resolve().parent / ".venv" / "bin")


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=[f"{VENV_BIN}/arm"], output="screen"),
        ExecuteProcess(cmd=[f"{VENV_BIN}/cam"], output="screen"),
        ExecuteProcess(cmd=[f"{VENV_BIN}/control-manager"], output="screen"),
        ExecuteProcess(cmd=[f"{VENV_BIN}/policy-client"], output="screen"),
        ExecuteProcess(cmd=[f"{VENV_BIN}/monitor"], output="screen"),
    ])
