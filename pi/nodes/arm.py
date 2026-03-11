import rclpy

from nodes._arm import ArmNode
from nodes._util import run_node


def main():
    rclpy.init()
    run_node(ArmNode())
