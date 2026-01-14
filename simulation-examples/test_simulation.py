"""
Testing framework for simulation examples in the Physical AI & Humanoid Robotics Course.

This module provides test cases for ROS 2 nodes, simulation controllers, and utility functions.
"""

import unittest
import pytest
import numpy as np
from unittest.mock import Mock, patch
import sys
import os

# Add the simulation scripts directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'python-scripts'))

from utilities import (
    RobotController, JointStatePublisher, PIDController,
    normalize_angle, distance_2d, quaternion_from_euler, euler_from_quaternion,
    clamp, RateLimiter
)

from simulation_controllers.basic_controller import BasicHumanoidController


class TestRobotController:
    """Test cases for the RobotController class."""

    def test_robot_controller_initialization(self):
        """Test that RobotController initializes correctly."""
        mock_node = Mock()
        controller = RobotController(mock_node, '/test_cmd_vel')

        assert controller.node == mock_node
        mock_node.create_publisher.assert_called_once()

    def test_move_linear(self):
        """Test linear movement functionality."""
        mock_node = Mock()
        controller = RobotController(mock_node, '/test_cmd_vel')

        # Mock the publisher
        controller.cmd_vel_publisher = Mock()

        controller.move_linear(x=1.0, y=0.5, z=0.0)

        # Check that publish was called
        assert controller.cmd_vel_publisher.publish.called

    def test_move_angular(self):
        """Test angular movement functionality."""
        mock_node = Mock()
        controller = RobotController(mock_node, '/test_cmd_vel')

        # Mock the publisher
        controller.cmd_vel_publisher = Mock()

        controller.move_angular(z=1.0)

        # Check that publish was called
        assert controller.cmd_vel_publisher.publish.called


class TestJointStatePublisher:
    """Test cases for the JointStatePublisher class."""

    def test_publish_joint_state(self):
        """Test that joint states are published correctly."""
        mock_node = Mock()
        publisher = JointStatePublisher(mock_node, '/test_joint_states')

        # Mock the publisher
        publisher.joint_state_publisher = Mock()

        joint_names = ['joint1', 'joint2']
        positions = [1.0, 2.0]

        publisher.publish_joint_state(joint_names, positions)

        # Check that publish was called
        assert publisher.joint_state_publisher.publish.called


class TestPIDController:
    """Test cases for the PIDController class."""

    def test_pid_initialization(self):
        """Test PID controller initialization."""
        pid = PIDController(kp=1.0, ki=0.5, kd=0.1, dt=0.01)

        assert pid.kp == 1.0
        assert pid.ki == 0.5
        assert pid.kd == 0.1
        assert pid.dt == 0.01

    def test_pid_compute(self):
        """Test PID computation."""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.01)

        # With zero error, output should be zero
        output = pid.compute(0.0)
        assert output == 0.0

        # With positive error, output should be positive
        output = pid.compute(1.0)
        assert output > 0.0

    def test_pid_integral_accumulation(self):
        """Test that integral term accumulates over time."""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=0.01)

        # First call
        output1 = pid.compute(1.0)

        # Second call with same error should have higher integral
        output2 = pid.compute(1.0)

        assert output2 > output1


class TestUtils:
    """Test cases for utility functions."""

    def test_normalize_angle(self):
        """Test angle normalization."""
        # Test normalizing angles outside [-pi, pi]
        assert abs(normalize_angle(4*np.pi) - 0) < 1e-10
        assert abs(normalize_angle(3*np.pi) - np.pi) < 1e-10
        assert abs(normalize_angle(-3*np.pi) - np.pi) < 1e-10

    def test_distance_2d(self):
        """Test 2D distance calculation."""
        dist = distance_2d((0, 0), (3, 4))
        assert abs(dist - 5.0) < 1e-10  # 3-4-5 triangle

        dist = distance_2d((1, 1), (1, 1))
        assert dist == 0.0

    def test_quaternion_from_euler(self):
        """Test quaternion conversion from Euler angles."""
        # Identity rotation
        w, x, y, z = quaternion_from_euler(0, 0, 0)
        assert abs(w - 1.0) < 1e-10
        assert abs(x) < 1e-10
        assert abs(y) < 1e-10
        assert abs(z) < 1e-10

        # 180 degree rotation around Z axis
        w, x, y, z = quaternion_from_euler(0, 0, np.pi)
        assert abs(w) < 1e-10
        assert abs(x) < 1e-10
        assert abs(y) < 1e-10
        assert abs(z - 1.0) < 1e-10

    def test_euler_from_quaternion(self):
        """Test Euler conversion from quaternion."""
        # Identity rotation
        roll, pitch, yaw = euler_from_quaternion(1, 0, 0, 0)
        assert abs(roll) < 1e-10
        assert abs(pitch) < 1e-10
        assert abs(yaw) < 1e-10

    def test_clamp(self):
        """Test value clamping."""
        assert clamp(5, 0, 10) == 5
        assert clamp(-5, 0, 10) == 0
        assert clamp(15, 0, 10) == 10


class TestBasicHumanoidController:
    """Test cases for the BasicHumanoidController class."""

    @patch('rclpy.node.Node')
    def test_controller_initialization(self, mock_node):
        """Test that the controller initializes with correct joint names."""
        controller = BasicHumanoidController()

        expected_joints = [
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint',
            'right_hip_joint', 'right_knee_joint'
        ]

        assert len(controller.current_positions) == len(expected_joints)
        assert len(controller.target_positions) == len(expected_joints)

        for joint in expected_joints:
            assert joint in controller.current_positions
            assert joint in controller.target_positions


def test_example():
    """Example test that always passes."""
    assert True


if __name__ == '__main__':
    # Run tests with pytest
    pytest.main([__file__, '-v'])