#!/usr/bin/env python3

"""
Comprehensive Capstone Simulation Example
This example demonstrates integration of all course concepts:
- ROS 2 fundamentals
- Simulation environment (Isaac Sim/Gazebo)
- Perception and navigation
- VLA (Vision-Language-Action) systems
- LLM integration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np
import math
import time
import threading
import queue
import json
from typing import Dict, List, Any, Optional


class CapstoneDemoNode(Node):
    """
    Comprehensive capstone demonstration node that integrates all course concepts.
    """

    def __init__(self):
        super().__init__('capstone_demo_node')

        # Create CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/capstone_demo/status', 10)
        self.voice_cmd_pub = self.create_publisher(String, '/capstone/voice_commands', 10)

        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )
        self.voice_resp_sub = self.create_subscription(
            String, '/capstone/language_response', self.voice_response_callback, 10
        )

        # State management
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.laser_data = None
        self.camera_data = None
        self.detections = []
        self.navigation_active = False
        self.voice_command_queue = queue.Queue()
        self.demo_state = 'waiting_for_command'  # waiting_for_command, executing, completed

        # Demo scenarios
        self.demo_scenarios = {
            'navigation_demo': self.navigation_demo,
            'perception_demo': self.perception_demo,
            'vla_demo': self.vla_demo,
            'full_integration_demo': self.full_integration_demo
        }

        # Timer for demo execution
        self.demo_timer = self.create_timer(0.1, self.demo_execution_loop)

        self.get_logger().info('Capstone Demo Node initialized and ready')

    def camera_callback(self, msg):
        """Process camera data for perception."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_data = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {e}')

    def lidar_callback(self, msg):
        """Process LIDAR data for navigation and obstacle detection."""
        self.laser_data = msg

    def detection_callback(self, msg):
        """Process object detections."""
        self.detections = msg.detections

    def voice_response_callback(self, msg):
        """Process voice command responses."""
        self.get_logger().info(f'Voice response received: {msg.data}')

        # Parse the response to determine next action
        try:
            response_data = json.loads(msg.data)
            if 'action_sequence' in response_data:
                self.execute_action_sequence(response_data['action_sequence'])
        except json.JSONDecodeError:
            # Simple text response
            if 'NAVIGATING' in msg.data.upper():
                self.demo_state = 'executing_navigation'
            elif 'DETECTING' in msg.data.upper():
                self.demo_state = 'executing_perception'

    def start_demo_scenario(self, scenario_name: str):
        """Start a specific demo scenario."""
        if scenario_name in self.demo_scenarios:
            self.get_logger().info(f'Starting demo scenario: {scenario_name}')

            # Set demo state
            self.demo_state = f'executing_{scenario_name}'

            # Execute the scenario
            self.demo_scenarios[scenario_name]()
        else:
            self.get_logger().error(f'Unknown demo scenario: {scenario_name}')

    def navigation_demo(self):
        """Execute navigation demonstration."""
        self.get_logger().info('Starting navigation demonstration')

        # Example: Navigate to specific locations
        navigation_goals = [
            {'x': 2.0, 'y': 2.0, 'theta': 0.0},
            {'x': -1.0, 'y': 3.0, 'theta': 1.57},
            {'x': -3.0, 'y': -1.0, 'theta': 3.14},
            {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # Return to start
        ]

        for i, goal in enumerate(navigation_goals):
            self.get_logger().info(f'Navigating to goal {i+1}: ({goal["x"]}, {goal["y"]})')

            # Send navigation goal
            self.send_navigation_goal(goal['x'], goal['y'], goal['theta'])

            # Wait for navigation to complete (simplified)
            time.sleep(3.0)  # In real implementation, monitor navigation status

        self.get_logger().info('Navigation demonstration completed')
        self.demo_state = 'completed_navigation_demo'

    def perception_demo(self):
        """Execute perception demonstration."""
        self.get_logger().info('Starting perception demonstration')

        # Process camera data to detect objects
        if self.camera_data is not None:
            self.get_logger().info('Processing camera data for object detection')

            # In a real implementation, this would use Isaac ROS perception
            # For this example, we'll just log that we have image data
            height, width, channels = self.camera_data.shape
            self.get_logger().info(f'Image processed: {width}x{height}x{channels}')

        # Process LIDAR data for environment mapping
        if self.laser_data:
            # Count obstacles in front of robot
            front_sector = self.laser_data.ranges[
                len(self.laser_data.ranges)//2-30:len(self.laser_data.ranges)//2+30
            ]
            obstacles = [r for r in front_sector if 0 < r < 1.0 and r < float('inf')]
            self.get_logger().info(f'Perception: {len(obstacles)} obstacles detected in front')

        self.get_logger().info('Perception demonstration completed')
        self.demo_state = 'completed_perception_demo'

    def vla_demo(self):
        """Execute Vision-Language-Action demonstration."""
        self.get_logger().info('Starting Vision-Language-Action demonstration')

        # Simulate receiving a voice command
        voice_commands = [
            "Move forward 1 meter",
            "Turn left 90 degrees",
            "Go to the kitchen",
            "Pick up the red cup"
        ]

        for cmd in voice_commands:
            self.get_logger().info(f'Simulating voice command: {cmd}')

            # Publish command to system
            cmd_msg = String()
            cmd_msg.data = cmd
            self.voice_cmd_pub.publish(cmd_msg)

            # Wait for processing
            time.sleep(2.0)

        self.get_logger().info('VLA demonstration completed')
        self.demo_state = 'completed_vla_demo'

    def full_integration_demo(self):
        """Execute full integration demonstration."""
        self.get_logger().info('Starting full integration demonstration')

        # Combine all capabilities
        self.perception_demo()
        time.sleep(1.0)

        self.navigation_demo()
        time.sleep(1.0)

        self.vla_demo()
        time.sleep(1.0)

        self.get_logger().info('Full integration demonstration completed')
        self.demo_state = 'completed_full_integration_demo'

    def send_navigation_goal(self, x: float, y: float, theta: float = 0.0):
        """Send a navigation goal to the system."""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        # Convert theta to quaternion
        goal_msg.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.orientation.w = math.cos(theta / 2.0)

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Navigation goal sent: ({x}, {y}, {theta})')

    def execute_action_sequence(self, action_sequence: List[Dict[str, Any]]):
        """Execute a sequence of actions."""
        self.get_logger().info(f'Executing action sequence: {action_sequence}')

        for action in action_sequence:
            action_type = action.get('type')
            params = action.get('parameters', {})

            if action_type == 'move':
                self.execute_move_action(params)
            elif action_type == 'turn':
                self.execute_turn_action(params)
            elif action_type == 'navigate':
                self.execute_navigate_action(params)
            elif action_type == 'detect':
                self.execute_detect_action(params)

            # Brief pause between actions
            time.sleep(0.5)

    def execute_move_action(self, params: Dict[str, Any]):
        """Execute a move action."""
        distance = params.get('distance', 1.0)
        direction = params.get('direction', 'forward')

        cmd = Twist()
        if direction == 'forward':
            cmd.linear.x = 0.3
        elif direction == 'backward':
            cmd.linear.x = -0.3
        else:
            return  # Invalid direction

        duration = distance / 0.3  # Assuming 0.3 m/s speed
        start_time = time.time()

        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def execute_turn_action(self, params: Dict[str, Any]):
        """Execute a turn action."""
        angle_deg = params.get('angle', 90)
        direction = params.get('direction', 'left')

        cmd = Twist()
        angular_speed = 0.5  # rad/s
        angle_rad = math.radians(angle_deg)
        duration = angle_rad / angular_speed

        if direction == 'left':
            cmd.angular.z = angular_speed
        elif direction == 'right':
            cmd.angular.z = -angular_speed
        else:
            return  # Invalid direction

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def execute_navigate_action(self, params: Dict[str, Any]):
        """Execute a navigation action."""
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        theta = params.get('theta', 0.0)

        self.send_navigation_goal(x, y, theta)

    def execute_detect_action(self, params: Dict[str, Any]):
        """Execute a detection action."""
        target_object = params.get('object', 'anything')

        self.get_logger().info(f'Detecting {target_object}')

        # In a real implementation, this would trigger perception processing
        # For this example, we'll just log the action
        if self.detections:
            self.get_logger().info(f'Found {len(self.detections)} detections')
        else:
            self.get_logger().info('No detections available')

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

    def demo_execution_loop(self):
        """Main demo execution loop."""
        status_msg = String()
        status_msg.data = f"STATE:{self.demo_state}|POSE:({self.current_pose['x']:.2f},{self.current_pose['y']:.2f})"
        self.status_pub.publish(status_msg)

        # In a real implementation, this would monitor the actual system state
        # For this example, we'll just log the current state periodically
        if int(time.time()) % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(f'Demo state: {self.demo_state}')

    def get_demo_status(self) -> Dict[str, Any]:
        """Get current demo status."""
        return {
            'state': self.demo_state,
            'current_pose': self.current_pose,
            'has_camera_data': self.camera_data is not None,
            'has_lidar_data': self.laser_data is not None,
            'detection_count': len(self.detections),
            'timestamp': time.time()
        }


class CapstoneDemoManager:
    """
    Manager for running capstone demonstrations.
    """

    def __init__(self, node: CapstoneDemoNode):
        self.node = node
        self.active_demo = None

    def run_navigation_demo(self):
        """Run the navigation demonstration."""
        self.node.start_demo_scenario('navigation_demo')

    def run_perception_demo(self):
        """Run the perception demonstration."""
        self.node.start_demo_scenario('perception_demo')

    def run_vla_demo(self):
        """Run the VLA demonstration."""
        self.node.start_demo_scenario('vla_demo')

    def run_full_integration_demo(self):
        """Run the full integration demonstration."""
        self.node.start_demo_scenario('full_integration_demo')

    def get_demo_status(self):
        """Get status of the current demo."""
        return self.node.get_demo_status()


def main(args=None):
    """Main function to run the capstone demonstration."""
    rclpy.init(args=args)

    demo_node = CapstoneDemoNode()
    demo_manager = CapstoneDemoManager(demo_node)

    try:
        # Example: Run the full integration demo
        demo_manager.run_full_integration_demo()

        # Run for a specific duration
        start_time = time.time()
        duration = 60  # Run for 60 seconds

        while time.time() - start_time < duration and rclpy.ok():
            rclpy.spin_once(demo_node, timeout_sec=0.1)

            # Print status periodically
            if int(time.time() - start_time) % 10 == 0:
                status = demo_manager.get_demo_status()
                demo_node.get_logger().info(f'Demo status: {status}')

    except KeyboardInterrupt:
        demo_node.get_logger().info('Capstone demo stopped by user')
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()