"""
test_zero_kinematics.py — reset /sensor_kinematics pose once
=============================================================
Minimal utility that sends ``SysOdomReset`` through ``Robot.reset_odometry()``
and prints the next pose update observed from the firmware.

Usage:
    ros2 run robot test_zero_kinematics

Prerequisite:
    ros2 run bridge bridge
"""

from __future__ import annotations

import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from robot.robot import Robot


def run(robot: Robot) -> None:
    print("[zero_kinematics] Publishing SysOdomReset via Robot.reset_odometry()")
    robot.reset_odometry()

    if not robot.wait_for_pose_update(timeout=1.0):
        raise RuntimeError(
            "[zero_kinematics] Timed out waiting for /sensor_kinematics after reset."
        )

    x, y, theta_deg = robot.get_pose()
    print(
        "[zero_kinematics] Pose after reset: "
        f"x={x:.2f}, y={y:.2f}, theta={theta_deg:.2f} deg"
    )


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    class _ZeroKinematicsNode(Node):
        def __init__(self) -> None:
            super().__init__("zero_kinematics_test")
            self.robot = Robot(self)

    node = _ZeroKinematicsNode()

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    def _raise_keyboard_interrupt(signum, frame):
        raise KeyboardInterrupt()

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    try:
        run(node.robot)
    except KeyboardInterrupt:
        node.get_logger().info("test interrupted; shutting down")
    finally:
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
