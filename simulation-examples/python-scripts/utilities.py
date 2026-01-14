"""
Common utility functions for ROS 2 integration in the Physical AI & Humanoid Robotics Course.

This module contains reusable functions and classes that facilitate ROS 2 development
and integration with simulation environments.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from typing import List, Tuple, Optional


class RobotController:
    """
    A generic robot controller class that can be used to control various types of robots.
    """

    def __init__(self, node: Node, cmd_vel_topic: str = '/cmd_vel'):
        self.node = node
        self.cmd_vel_publisher = node.create_publisher(Twist, cmd_vel_topic, 10)

    def move_linear(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """Send linear velocity commands."""
        twist_msg = Twist()
        twist_msg.linear = Vector3(x=x, y=y, z=z)
        self.cmd_vel_publisher.publish(twist_msg)

    def move_angular(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """Send angular velocity commands."""
        twist_msg = Twist()
        twist_msg.angular = Vector3(x=x, y=y, z=z)
        self.cmd_vel_publisher.publish(twist_msg)


class JointStatePublisher:
    """
    Utility class for publishing joint state messages.
    """

    def __init__(self, node: Node, joint_state_topic: str = '/joint_states'):
        self.node = node
        self.joint_state_publisher = node.create_publisher(JointState, joint_state_topic, 10)

    def publish_joint_state(self, joint_names: List[str], positions: List[float],
                           velocities: Optional[List[float]] = None,
                           efforts: Optional[List[float]] = None):
        """Publish joint state message with given parameters."""
        msg = JointState()
        msg.name = joint_names
        msg.position = positions
        msg.velocity = velocities if velocities is not None else [0.0] * len(positions)
        msg.effort = efforts if efforts is not None else [0.0] * len(positions)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_state_publisher.publish(msg)


class PIDController:
    """
    Simple PID Controller implementation for robot control.
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0, dt: float = 0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, error: float) -> float:
        """Compute PID output based on error."""
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.previous_error = error

        return output


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def distance_2d(point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
    """Calculate 2D Euclidean distance between two points."""
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles to quaternion."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)


def euler_from_quaternion(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a value between min and max."""
    return max(min_val, min(max_val, value))


class RateLimiter:
    """
    Utility class to limit execution rate of functions.
    """

    def __init__(self, node: Node, hz: float):
        self.node = node
        self.rate = node.create_rate(hz)

    def sleep(self):
        """Sleep for the appropriate amount of time to maintain the rate."""
        self.rate.sleep()


def create_main_loop(node: Node, callback_func, rate_hz: float = 50):
    """
    Create a main loop that runs at a specified rate.

    Args:
        node: ROS 2 node instance
        callback_func: Function to call in the loop
        rate_hz: Rate in Hz
    """
    rate = node.create_rate(rate_hz)

    while rclpy.ok():
        callback_func()
        rate.sleep()


def shutdown_cleanup():
    """Perform cleanup operations before shutting down."""
    rclpy.shutdown()


if __name__ == "__main__":
    # Example usage
    print("Utility functions module loaded successfully.")
    print("This module provides common ROS 2 integration utilities for the Physical AI & Humanoid Robotics Course.")