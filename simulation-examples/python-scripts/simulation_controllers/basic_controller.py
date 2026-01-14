"""
Basic Simulation Controller for the Physical AI & Humanoid Robotics Course.

This module implements a basic controller for humanoid robot simulation that demonstrates
core control concepts for joints and movement.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
from typing import List, Dict, Optional


class BasicHumanoidController(Node):
    """
    Basic controller for humanoid robot simulation.
    Demonstrates joint control, inverse kinematics basics, and movement patterns.
    """

    def __init__(self):
        super().__init__('basic_humanoid_controller')

        # Joint names for the simple humanoid
        self.joint_names = [
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint',
            'right_hip_joint', 'right_knee_joint'
        ]

        # Publishers for different control interfaces
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop

        # Initialize joint positions
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.target_positions = {name: 0.0 for name in self.joint_names}

        self.get_logger().info('Basic Humanoid Controller initialized')

    def joint_state_callback(self, msg: JointState):
        """Callback to update current joint positions."""
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]

    def set_joint_positions(self, positions: Dict[str, float]):
        """Set target positions for specified joints."""
        for joint_name, position in positions.items():
            if joint_name in self.target_positions:
                self.target_positions[joint_name] = position

    def send_joint_commands(self):
        """Send current target positions as joint commands."""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.target_positions[name] for name in self.joint_names]
        self.joint_cmd_pub.publish(cmd_msg)

    def send_joint_trajectory(self, positions: List[float], duration: float = 1.0):
        """Send a joint trajectory command."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        traj_msg.points.append(point)
        self.joint_trajectory_pub.publish(traj_msg)

    def control_loop(self):
        """Main control loop - override this method to implement specific behaviors."""
        # Send current target positions
        self.send_joint_commands()

    def wave_motion(self):
        """Create a simple waving motion with the right arm."""
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds

        # Wave right arm
        right_shoulder_pos = math.sin(t) * 0.5
        right_elbow_pos = math.sin(t * 1.5) * 0.3

        self.target_positions['right_shoulder_joint'] = right_shoulder_pos
        self.target_positions['right_elbow_joint'] = right_elbow_pos

    def walk_motion(self):
        """Create a simple walking motion."""
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds

        # Simple leg motion to simulate walking
        left_hip_pos = math.sin(t) * 0.3
        left_knee_pos = math.sin(t + 0.5) * 0.2
        right_hip_pos = math.sin(t + math.pi) * 0.3
        right_knee_pos = math.sin(t + math.pi + 0.5) * 0.2

        self.target_positions['left_hip_joint'] = left_hip_pos
        self.target_positions['left_knee_joint'] = left_knee_pos
        self.target_positions['right_hip_joint'] = right_hip_pos
        self.target_positions['right_knee_joint'] = right_knee_pos

    def idle_pose(self):
        """Reset to an idle pose."""
        for joint_name in self.joint_names:
            self.target_positions[joint_name] = 0.0

    def dance_motion(self):
        """Create a simple dancing motion."""
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds

        # Arms moving in opposite directions
        left_arm_pos = math.sin(t * 2) * 0.7
        right_arm_pos = math.sin(t * 2 + math.pi) * 0.7

        # Legs moving in sync
        leg_pos = math.sin(t * 1.5) * 0.3

        self.target_positions['left_shoulder_joint'] = left_arm_pos
        self.target_positions['right_shoulder_joint'] = right_arm_pos
        self.target_positions['left_hip_joint'] = leg_pos
        self.target_positions['right_hip_joint'] = leg_pos


class PIDJointController(BasicHumanoidController):
    """
    Enhanced controller with PID control for each joint.
    """

    def __init__(self):
        super().__init__()

        # PID parameters for each joint
        self.pid_params = {
            name: {'kp': 10.0, 'ki': 0.1, 'kd': 0.5} for name in self.joint_names
        }

        # PID state storage
        self.pid_state = {
            name: {'prev_error': 0.0, 'integral': 0.0} for name in self.joint_names
        }

    def compute_pid_control(self, joint_name: str, current_pos: float, target_pos: float) -> float:
        """Compute PID control output for a single joint."""
        params = self.pid_params[joint_name]
        state = self.pid_state[joint_name]

        error = target_pos - current_pos

        # Integral term
        state['integral'] += error * 0.05  # dt = 0.05s from control loop

        # Derivative term
        derivative = (error - state['prev_error']) / 0.05

        # Store current error for next iteration
        state['prev_error'] = error

        # PID calculation
        output = params['kp'] * error + params['ki'] * state['integral'] + params['kd'] * derivative

        return output

    def control_loop(self):
        """Enhanced control loop with PID control."""
        # For each joint, compute PID control output
        pid_outputs = []
        for joint_name in self.joint_names:
            current_pos = self.current_positions.get(joint_name, 0.0)
            target_pos = self.target_positions[joint_name]

            control_output = self.compute_pid_control(joint_name, current_pos, target_pos)
            pid_outputs.append(control_output)

        # Publish the control commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.target_positions[name] for name in self.joint_names]
        self.joint_cmd_pub.publish(cmd_msg)


def main(args=None):
    """Main function to run the basic humanoid controller."""
    rclpy.init(args=args)

    controller = BasicHumanoidController()

    # Example: Set a simple waving motion
    def wave_behavior():
        controller.wave_motion()

    # Set timer for behavior
    controller.create_timer(0.1, wave_behavior)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller stopped by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()