#!/usr/bin/env python3

"""
Voice Navigation Example for VLA Demo
This example demonstrates voice-controlled navigation in Isaac Sim.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
import math
import time
import threading
import queue
import whisper
import openai
import json


class VoiceNavigationNode(Node):
    """
    A ROS 2 node that demonstrates voice-controlled navigation.
    """

    def __init__(self):
        super().__init__('voice_navigation_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/voice_nav_status', 10)

        # Subscribers
        self.voice_cmd_sub = self.create_subscription(
            String, '/voice_commands', self.voice_command_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Initialize Whisper model for local speech recognition
        self.get_logger().info('Loading Whisper model...')
        try:
            self.whisper_model = whisper.load_model("base")
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().warn(f'Could not load Whisper model: {e}')
            self.whisper_model = None

        # Robot state
        self.laser_data = None
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.is_navigating = False

        # Known locations
        self.known_locations = {
            'kitchen': [4.0, 3.0, 0.0],
            'living_room': [0.0, 0.0, 0.0],
            'bedroom': [-3.0, 2.0, 1.57],
            'office': [2.0, -3.0, 3.14],
            'entrance': [0.0, 4.0, 0.0]
        }

        # Command queue for processing
        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(target=self.process_commands)
        self.command_thread.daemon = True
        self.command_thread.start()

        self.get_logger().info('Voice Navigation Node initialized')

    def scan_callback(self, msg):
        """Process LIDAR data for obstacle detection."""
        self.laser_data = msg

    def voice_command_callback(self, msg):
        """Process voice command from Whisper or external source."""
        command_text = msg.data.lower()
        self.get_logger().info(f'Received voice command: {command_text}')

        # Add to processing queue
        self.command_queue.put(command_text)

    def process_commands(self):
        """Process commands from the queue in a separate thread."""
        while rclpy.ok():
            try:
                if not self.command_queue.empty():
                    command_text = self.command_queue.get_nowait()

                    # Process the command
                    self.execute_voice_command(command_text)

                time.sleep(0.1)  # Brief pause to prevent busy waiting
            except queue.Empty:
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'Error processing command: {e}')

    def execute_voice_command(self, command_text: str):
        """Execute a voice command."""
        self.get_logger().info(f'Processing voice command: {command_text}')

        # Check for navigation commands
        if 'go to' in command_text or 'navigate to' in command_text:
            self.handle_navigation_command(command_text)
        elif 'move forward' in command_text or 'go forward' in command_text:
            self.move_forward()
        elif 'move backward' in command_text or 'go back' in command_text:
            self.move_backward()
        elif 'turn left' in command_text:
            self.turn_left()
        elif 'turn right' in command_text:
            self.turn_right()
        elif 'stop' in command_text:
            self.stop_robot()
        elif 'help' in command_text:
            self.provide_help()
        else:
            # Try to extract location from command
            self.handle_potential_location_command(command_text)

    def handle_navigation_command(self, command_text: str):
        """Handle navigation commands."""
        # Extract location from command
        target_location = None
        for location in self.known_locations:
            if location in command_text:
                target_location = location
                break

        if target_location:
            self.navigate_to_location(target_location)
        else:
            # If no known location found, try to parse coordinates
            coords = self.parse_coordinates(command_text)
            if coords:
                self.navigate_to_coordinates(coords[0], coords[1])
            else:
                self.get_logger().warn(f'Unknown location in command: {command_text}')
                status_msg = String()
                status_msg.data = f"UNKNOWN_LOCATION: {command_text}"
                self.status_pub.publish(status_msg)

    def handle_potential_location_command(self, command_text: str):
        """Handle potential location commands that don't use 'go to' pattern."""
        for location in self.known_locations:
            if location in command_text:
                self.navigate_to_location(location)
                return

        # If no location found, try to use LLM for interpretation
        self.interpret_with_llm(command_text)

    def navigate_to_location(self, location_name: str):
        """Navigate to a known location."""
        if location_name in self.known_locations:
            target_pos = self.known_locations[location_name]
            self.get_logger().info(f'Navigating to {location_name} at ({target_pos[0]}, {target_pos[1]})')

            # Check if path is clear using LIDAR data
            if self.laser_data:
                obstacles = self.laser_data.ranges
                front_sector = obstacles[len(obstacles)//2-30:len(obstacles)//2+30]

                # Check if path is clear in front of robot
                min_distance = min([r for r in front_sector if r > 0 and r < float('inf')], default=float('inf'))

                if min_distance < 0.8:  # Obstacle too close
                    self.get_logger().warn(f'Path to {location_name} blocked by obstacle at {min_distance:.2f}m')
                    status_msg = String()
                    status_msg.data = f"PATH_BLOCKED_TO_{location_name.upper()}:obstacle_at_{min_distance:.2f}m"
                    self.status_pub.publish(status_msg)
                    return

            # Create navigation goal
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            goal_msg.pose.position.x = target_pos[0]
            goal_msg.pose.position.y = target_pos[1]
            goal_msg.pose.position.z = 0.0

            # Convert theta to quaternion
            theta = target_pos[2]
            goal_msg.pose.orientation.z = math.sin(theta / 2.0)
            goal_msg.pose.orientation.w = math.cos(theta / 2.0)

            self.nav_goal_pub.publish(goal_msg)

            # Update status
            status_msg = String()
            status_msg.data = f"NAVIGATING_TO: {location_name}"
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().warn(f'Unknown location: {location_name}')

    def navigate_to_coordinates(self, x: float, y: float):
        """Navigate to specific coordinates."""
        self.get_logger().info(f'Navigating to coordinates: ({x}, {y})')

        # Create navigation goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        # Use current orientation
        current_theta = self.robot_pose[2]
        goal_msg.pose.orientation.z = math.sin(current_theta / 2.0)
        goal_msg.pose.orientation.w = math.cos(current_theta / 2.0)

        self.nav_goal_pub.publish(goal_msg)

        # Update status
        status_msg = String()
        status_msg.data = f"NAVIGATING_TO_COORDS: ({x}, {y})"
        self.status_pub.publish(status_msg)

    def move_forward(self):
        """Move robot forward."""
        self.get_logger().info('Moving forward')

        cmd = Twist()
        cmd.linear.x = 0.3  # Moderate speed
        cmd.angular.z = 0.0

        # Move for 2 seconds
        start_time = time.time()
        while time.time() - start_time < 2.0 and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def move_backward(self):
        """Move robot backward."""
        self.get_logger().info('Moving backward')

        cmd = Twist()
        cmd.linear.x = -0.3  # Backward
        cmd.angular.z = 0.0

        # Move for 1 second
        start_time = time.time()
        while time.time() - start_time < 1.0 and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def turn_left(self):
        """Turn robot left."""
        self.get_logger().info('Turning left')

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Positive for counter-clockwise

        # Turn for 1 second (90 degrees at 0.5 rad/s)
        start_time = time.time()
        while time.time() - start_time < 1.0 and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def turn_right(self):
        """Turn robot right."""
        self.get_logger().info('Turning right')

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.5  # Negative for clockwise

        # Turn for 1 second (90 degrees at 0.5 rad/s)
        start_time = time.time()
        while time.time() - start_time < 1.0 and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def stop_robot(self):
        """Stop robot movement."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Robot stopped')

    def parse_coordinates(self, command: str) -> tuple:
        """Parse coordinates from command if present."""
        # Simple coordinate parsing - look for numbers in command
        import re
        numbers = re.findall(r'-?\d+\.?\d*', command)

        if len(numbers) >= 2:
            try:
                x = float(numbers[0])
                y = float(numbers[1])
                return (x, y)
            except ValueError:
                pass

        return None

    def interpret_with_llm(self, command_text: str):
        """Use LLM to interpret complex commands."""
        # In a real implementation, this would call an LLM API
        # For this example, we'll implement a simple rule-based interpretation
        self.get_logger().info(f'Interpreting command with LLM: {command_text}')

        # Simple rule-based interpretation for demonstration
        if 'bring me' in command_text or 'get me' in command_text:
            # Extract object if possible
            for obj in ['coffee', 'water', 'book', 'cup']:
                if obj in command_text:
                    self.get_logger().info(f'Command interpreted as fetching {obj}')
                    status_msg = String()
                    status_msg.data = f"INTERPRETED_FETCH_COMMAND: {obj}"
                    self.status_pub.publish(status_msg)
                    return

            self.get_logger().info('Command interpreted as fetching an object')
            status_msg = String()
            status_msg.data = "INTERPRETED_FETCH_COMMAND: unknown_object"
            self.status_pub.publish(status_msg)

        elif 'follow me' in command_text or 'come with me' in command_text:
            self.get_logger().info('Command interpreted as following')
            status_msg = String()
            status_msg.data = "INTERPRETED_FOLLOW_COMMAND"
            self.status_pub.publish(status_msg)

        else:
            self.get_logger().info(f'Command not understood: {command_text}')
            status_msg = String()
            status_msg.data = f"COMMAND_NOT_UNDERSTOOD: {command_text}"
            self.status_pub.publish(status_msg)

    def provide_help(self):
        """Provide help information."""
        help_text = (
            "I understand commands like: "
            "'go to kitchen', 'move forward', 'turn left', 'stop', "
            "'bring me coffee', 'follow me', 'what can you do'"
        )

        self.get_logger().info(f'Providing help: {help_text}')
        status_msg = String()
        status_msg.data = f"HELP_INFO: {help_text}"
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean up resources before shutting down."""
        super().destroy_node()
        self.get_logger().info('Voice Navigation Node destroyed')


def main(args=None):
    """Main function to run the voice navigation node."""
    rclpy.init(args=args)

    voice_nav_node = VoiceNavigationNode()

    try:
        rclpy.spin(voice_nav_node)
    except KeyboardInterrupt:
        voice_nav_node.get_logger().info('Voice navigation node stopped by user')
    finally:
        voice_nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()