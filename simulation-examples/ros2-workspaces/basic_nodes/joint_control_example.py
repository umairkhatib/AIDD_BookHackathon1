#!/usr/bin/env python3

"""
Joint Control Example for the Physical AI & Humanoid Robotics Course
This example demonstrates how to control robot joints using ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
from collections import deque


class JointController(Node):
    """
    Joint controller node that demonstrates various control strategies for robot joints.
    """

    def __init__(self):
        super().__init__('joint_controller')

        # Joint names for a simple robot (e.g., 6-DOF arm)
        self.joint_names = [
            'shoulder_joint', 'elbow_joint', 'wrist_joint',
            'joint4', 'joint5', 'joint6'
        ]

        # Publishers
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
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        # Initialize joint positions
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.target_positions = {name: 0.0 for name in self.joint_names}
        self.velocity_limits = {name: 1.0 for name in self.joint_names}  # rad/s
        self.effort_limits = {name: 10.0 for name in self.joint_names}  # Nm

        # Store position history for analysis
        self.position_history = {name: deque(maxlen=100) for name in self.joint_names}

        # Control mode: 'position', 'trajectory', or 'pid'
        self.control_mode = 'position'

        # PID controller parameters
        self.pid_params = {
            name: {'kp': 10.0, 'ki': 0.1, 'kd': 0.5} for name in self.joint_names
        }
        self.pid_state = {
            name: {'prev_error': 0.0, 'integral': 0.0} for name in self.joint_names
        }

        self.get_logger().info('Joint Controller initialized')
        self.get_logger().info(f'Controlling joints: {self.joint_names}')

    def joint_state_callback(self, msg: JointState):
        """Callback to update current joint positions."""
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]
                self.position_history[name].append(msg.position[i])

    def set_joint_positions(self, positions_dict):
        """Set target positions for specified joints."""
        for joint_name, position in positions_dict.items():
            if joint_name in self.target_positions:
                self.target_positions[joint_name] = position

    def send_joint_commands(self):
        """Send current target positions as joint commands."""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.target_positions[name] for name in self.joint_names]
        self.joint_cmd_pub.publish(cmd_msg)

    def send_joint_trajectory(self, positions, duration=1.0):
        """Send a joint trajectory command."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        traj_msg.points.append(point)
        self.joint_trajectory_pub.publish(traj_msg)

    def compute_pid_control(self, joint_name, current_pos, target_pos):
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
        """Main control loop."""
        if self.control_mode == 'position':
            # Direct position control
            self.send_joint_commands()
        elif self.control_mode == 'pid':
            # PID-based position control
            pid_outputs = []
            for joint_name in self.joint_names:
                current_pos = self.current_positions.get(joint_name, 0.0)
                target_pos = self.target_positions[joint_name]

                control_output = self.compute_pid_control(joint_name, current_pos, target_pos)
                pid_outputs.append(control_output)

            # Publish PID control commands (this would be the control signal, not position)
            cmd_msg = Float64MultiArray()
            cmd_msg.data = pid_outputs  # In a real system, this might go to a different topic
            # For this example, we'll still send position commands
            pos_cmd = Float64MultiArray()
            pos_cmd.data = [self.target_positions[name] for name in self.joint_names]
            self.joint_cmd_pub.publish(pos_cmd)

    def move_to_home_position(self):
        """Move all joints to home position (0.0)."""
        home_positions = {name: 0.0 for name in self.joint_names}
        self.set_joint_positions(home_positions)
        self.get_logger().info('Moving to home position')

    def move_to_test_positions(self):
        """Move to test positions for demonstration."""
        test_positions = {
            'shoulder_joint': 0.5,
            'elbow_joint': -0.3,
            'wrist_joint': 0.8,
            'joint4': -0.2,
            'joint5': 0.4,
            'joint6': -0.1
        }
        self.set_joint_positions(test_positions)
        self.get_logger().info('Moving to test positions')

    def execute_sine_wave_motion(self):
        """Execute a sine wave motion pattern."""
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds

        sine_positions = {}
        for i, name in enumerate(self.joint_names):
            # Different frequencies for each joint
            freq = 0.5 + i * 0.2
            sine_positions[name] = 0.5 * math.sin(freq * t)

        self.set_joint_positions(sine_positions)

    def execute_circular_motion(self):
        """Execute a circular motion with the end effector (simplified)."""
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds

        # This is a simplified example - in a real robot you'd need inverse kinematics
        shoulder_pos = 0.5 * math.sin(t)
        elbow_pos = 0.3 * math.cos(t * 1.5)
        wrist_pos = 0.4 * math.sin(t * 0.7)

        circular_positions = {
            'shoulder_joint': shoulder_pos,
            'elbow_joint': elbow_pos,
            'wrist_joint': wrist_pos
        }
        self.set_joint_positions(circular_positions)

    def get_joint_info(self):
        """Print current joint information."""
        self.get_logger().info("Current Joint States:")
        for name in self.joint_names:
            current_pos = self.current_positions.get(name, 0.0)
            target_pos = self.target_positions[name]
            self.get_logger().info(f"  {name}: Current={current_pos:.3f}, Target={target_pos:.3f}")


def main(args=None):
    """Main function to run the joint controller."""
    rclpy.init(args=args)

    controller = JointController()

    # Set a timer to execute different motions
    motion_counter = 0

    def motion_timer_callback():
        nonlocal motion_counter
        motion_counter += 1

        if motion_counter < 50:  # First 2.5 seconds: home position
            controller.move_to_home_position()
        elif motion_counter < 100:  # Next 2.5 seconds: test positions
            controller.move_to_test_positions()
        elif motion_counter < 150:  # Next 2.5 seconds: sine wave
            controller.execute_sine_wave_motion()
        elif motion_counter < 200:  # Next 2.5 seconds: circular
            controller.execute_circular_motion()
        else:  # Reset counter
            motion_counter = 0

        # Print joint info every 20 iterations (1 second)
        if motion_counter % 20 == 0:
            controller.get_joint_info()

    # Create timer for motion changes
    controller.create_timer(0.05, motion_timer_callback)  # 20Hz

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Joint controller stopped by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()