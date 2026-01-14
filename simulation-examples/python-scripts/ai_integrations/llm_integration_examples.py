#!/usr/bin/env python3

"""
LLM Integration Examples for Physical AI & Humanoid Robotics Course
This module demonstrates integration of Large Language Models (LLMs) with robotic systems.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import openai
import whisper
import json
import time
import numpy as np
from typing import Dict, List, Any, Optional
import asyncio
import threading
import queue


class LLMIntegrationNode(Node):
    """
    A ROS 2 node that demonstrates LLM integration with robotic systems.
    This node shows how to use LLMs to interpret natural language commands
    and convert them to robot actions.
    """

    def __init__(self):
        super().__init__('llm_integration_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/llm_status', 10)
        self.action_pub = self.create_publisher(String, '/robot_action', 10)

        # Subscribers
        self.voice_cmd_sub = self.create_subscription(
            String, '/voice_commands', self.voice_command_callback, 10
        )
        self.text_cmd_sub = self.create_subscription(
            String, '/text_commands', self.text_command_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10
        )

        # Initialize components
        self.cv_bridge = CvBridge()

        # Robot state and context
        self.laser_data = None
        self.camera_data = None
        self.robot_pose = None
        self.environment_context = {
            'obstacles': [],
            'free_spaces': [],
            'known_objects': [],
            'last_command_time': time.time()
        }

        # LLM configuration
        # Note: In a real implementation, you would set OPENAI_API_KEY in environment
        # self.openai_client = openai.OpenAI(api_key=os.environ.get('OPENAI_API_KEY'))

        # For demonstration purposes, we'll use a mock LLM interface
        self.use_mock_llm = True

        # Known locations and objects for demonstration
        self.known_locations = {
            'kitchen': {'x': 5.0, 'y': 3.0, 'theta': 0.0},
            'living_room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'bedroom': {'x': -3.0, 'y': 2.0, 'theta': 1.57},
            'office': {'x': 2.0, 'y': -2.0, 'theta': 3.14}
        }

        self.known_objects = [
            'cup', 'bottle', 'chair', 'table', 'book', 'box'
        ]

        # Command processing queue
        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(target=self.process_command_queue)
        self.command_thread.daemon = True
        self.command_thread.start()

        self.get_logger().info('LLM Integration Node initialized')

    def scan_callback(self, msg):
        """Process LIDAR data for environmental context."""
        self.laser_data = msg

        # Update environment context with obstacle information
        if msg.ranges:
            obstacles = []
            free_spaces = []

            for i, range_val in enumerate(msg.ranges):
                if 0 < range_val < float('inf'):
                    angle = msg.angle_min + i * msg.angle_increment
                    pos_info = {
                        'distance': range_val,
                        'angle': angle,
                        'x': range_val * np.cos(angle),
                        'y': range_val * np.sin(angle)
                    }

                    if 0 < range_val < 1.0:  # Obstacles within 1 meter
                        obstacles.append(pos_info)
                    elif range_val > 2.0:  # Free space beyond 2 meters
                        free_spaces.append(pos_info)

            self.environment_context['obstacles'] = obstacles
            self.environment_context['free_spaces'] = free_spaces

    def camera_callback(self, msg):
        """Process camera data for visual context."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_data = cv_image

            # In a real implementation, you would run object detection here
            # For this example, we'll just store the image
            self.environment_context['has_camera_data'] = True

        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {e}')

    def voice_command_callback(self, msg):
        """Process voice commands from Whisper."""
        self.get_logger().info(f'Received voice command: {msg.data}')
        self.process_command(msg.data)

    def text_command_callback(self, msg):
        """Process text commands."""
        self.get_logger().info(f'Received text command: {msg.data}')
        self.process_command(msg.data)

    def process_command(self, command_text: str):
        """Add command to processing queue."""
        command_entry = {
            'text': command_text,
            'timestamp': time.time(),
            'source': 'voice' if ' ' in command_text else 'text'  # Simplified source detection
        }
        self.command_queue.put(command_entry)

    def process_command_queue(self):
        """Process commands from the queue in a separate thread."""
        while rclpy.ok():
            try:
                if not self.command_queue.empty():
                    command_entry = self.command_queue.get_nowait()

                    # Process the command with LLM
                    action_sequence = self.interpret_with_llm(command_entry['text'])

                    if action_sequence:
                        self.execute_action_sequence(action_sequence)
                    else:
                        self.get_logger().warn(f'Could not interpret command: {command_entry["text"]}')

                        # Publish error status
                        status_msg = String()
                        status_msg.data = f"ERROR_INTERPRETING_COMMAND: {command_entry['text']}"
                        self.status_pub.publish(status_msg)

                time.sleep(0.1)  # Brief pause to prevent busy waiting
            except queue.Empty:
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'Error processing command queue: {e}')

    def interpret_with_llm(self, command_text: str) -> Optional[List[Dict[str, Any]]]:
        """
        Interpret the command using an LLM and return an action sequence.
        In a real implementation, this would call an LLM API.
        For this example, we'll use a mock implementation.
        """
        self.get_logger().info(f'Interpreting command with LLM: {command_text}')

        if self.use_mock_llm:
            return self.mock_llm_interpretation(command_text)
        else:
            return self.real_llm_interpretation(command_text)

    def mock_llm_interpretation(self, command_text: str) -> Optional[List[Dict[str, Any]]]:
        """
        Mock LLM interpretation for demonstration purposes.
        In a real implementation, this would call an actual LLM.
        """
        command_lower = command_text.lower()

        # Simple rule-based mapping that mimics LLM behavior
        if 'move forward' in command_lower or 'go forward' in command_lower or 'move ahead' in command_lower:
            return [{'action': 'move', 'direction': 'forward', 'distance': 1.0}]
        elif 'move backward' in command_lower or 'go back' in command_lower:
            return [{'action': 'move', 'direction': 'backward', 'distance': 1.0}]
        elif 'turn left' in command_lower:
            return [{'action': 'turn', 'direction': 'left', 'angle': 90}]
        elif 'turn right' in command_lower:
            return [{'action': 'turn', 'direction': 'right', 'angle': 90}]
        elif 'stop' in command_lower or 'halt' in command_lower:
            return [{'action': 'stop'}]
        elif 'spin left' in command_lower:
            return [{'action': 'spin', 'direction': 'left', 'angle': 360}]
        elif 'spin right' in command_lower:
            return [{'action': 'spin', 'direction': 'right', 'angle': 360}]
        elif 'go to' in command_lower:
            # Extract location from command
            for location in self.known_locations:
                if location in command_lower:
                    return [
                        {'action': 'navigate', 'location': location},
                        {'action': 'arrive', 'location': location}
                    ]
            # If no known location found, return navigate with destination
            return [{'action': 'navigate', 'destination': 'unknown'}]
        elif 'follow' in command_lower or 'track' in command_lower:
            return [{'action': 'follow', 'target': 'person'}]
        elif 'bring' in command_lower or 'fetch' in command_lower or 'get' in command_lower:
            # Extract object from command
            for obj in self.known_objects:
                if obj in command_lower:
                    return [
                        {'action': 'find_object', 'object': obj},
                        {'action': 'approach_object', 'object': obj},
                        {'action': 'grasp_object', 'object': obj},
                        {'action': 'return_to_user'},
                        {'action': 'release_object', 'object': obj}
                    ]
            # If no known object found, return generic fetch action
            return [{'action': 'fetch', 'object': 'unknown'}]
        elif 'describe' in command_lower or 'what do you see' in command_lower:
            return [{'action': 'describe_environment'}]
        elif 'take picture' in command_lower or 'photo' in command_lower:
            return [{'action': 'capture_image'}]
        elif 'help' in command_lower:
            return [{'action': 'provide_help'}]
        else:
            # For complex commands, try to break them down
            actions = []

            if 'and' in command_lower:
                # Split complex commands
                sub_commands = command_text.split(' and ')
                for sub_cmd in sub_commands:
                    sub_actions = self.mock_llm_interpretation(sub_cmd.strip())
                    if sub_actions:
                        actions.extend(sub_actions)
            elif 'then' in command_lower:
                # Handle sequential commands
                sub_commands = command_text.split(' then ')
                for sub_cmd in sub_commands:
                    sub_actions = self.mock_llm_interpretation(sub_cmd.strip())
                    if sub_actions:
                        actions.extend(sub_actions)
            else:
                self.get_logger().info(f'Command not recognized: {command_text}')
                return None

            return actions if actions else None

    def real_llm_interpretation(self, command_text: str) -> Optional[List[Dict[str, Any]]]:
        """
        Real LLM interpretation using OpenAI API.
        This would be used in a production implementation.
        """
        try:
            # Construct the prompt for the LLM
            prompt = self.construct_llm_prompt(command_text)

            # Call the LLM API
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.get_llm_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=500
            )

            # Parse the response
            response_text = response.choices[0].message.content

            # Extract structured actions from the response
            return self.parse_llm_response(response_text)

        except Exception as e:
            self.get_logger().error(f'Error calling LLM API: {e}')
            return None

    def construct_llm_prompt(self, command_text: str) -> str:
        """Construct a prompt for the LLM with context."""
        context = self.get_environment_context()

        prompt = f"""
        You are a command interpreter for a mobile robot. The robot can perform the following actions:

        - move(direction, distance): Move forward/backward/left/right
        - turn(direction, angle): Turn left/right by angle in degrees
        - navigate(location): Navigate to a known location
        - stop(): Stop all movement
        - follow(target): Follow a person or object
        - find_object(object): Locate a specific object
        - grasp_object(object): Pick up an object
        - release_object(object): Release an object
        - describe_environment(): Describe what the robot sees
        - capture_image(): Take a photo
        - provide_help(): Provide help information

        Known locations: {list(self.known_locations.keys())}
        Known objects: {self.known_objects}
        Environmental context: {json.dumps(context)}

        User command: "{command_text}"

        Respond with a JSON array of actions to execute. Each action should be a dictionary with 'action' as the key
        and any required parameters. For example:
        [
            {{"action": "navigate", "location": "kitchen"}},
            {{"action": "find_object", "object": "cup"}}
        ]

        Respond only with the JSON array, nothing else.
        """

        return prompt

    def get_llm_system_prompt(self) -> str:
        """Get the system prompt for the LLM."""
        return """
        You are a command interpreter for a mobile robot. Your role is to convert natural language commands
        into structured actions that the robot can execute. Be precise, safe, and only return actions that
        the robot is capable of performing. If a command is unsafe or impossible, return an empty array.
        Always respond with a JSON array of actions.
        """

    def parse_llm_response(self, response_text: str) -> Optional[List[Dict[str, Any]]]:
        """Parse the LLM response into structured actions."""
        try:
            # Clean the response to extract JSON
            start_idx = response_text.find('[')
            end_idx = response_text.rfind(']') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                actions = json.loads(json_str)

                # Validate actions
                for action in actions:
                    if 'action' not in action:
                        self.get_logger().warn(f'Invalid action format: {action}')
                        return None

                return actions
            else:
                self.get_logger().warn(f'Could not find JSON in LLM response: {response_text}')
                return None

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing LLM response as JSON: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Unexpected error parsing LLM response: {e}')
            return None

    def get_environment_context(self) -> Dict[str, Any]:
        """Get the current environment context for LLM."""
        context = {
            'robot_pose': self.robot_pose,
            'obstacles_count': len(self.environment_context.get('obstacles', [])),
            'known_objects': self.environment_context.get('known_objects', []),
            'last_command_time': self.environment_context['last_command_time'],
            'time_of_day': time.strftime('%H:%M', time.localtime()),
            'battery_level': 85.0  # Simulated battery level
        }
        return context

    def execute_action_sequence(self, action_sequence: List[Dict[str, Any]]):
        """Execute a sequence of actions."""
        self.get_logger().info(f'Executing action sequence: {action_sequence}')

        for action in action_sequence:
            action_type = action.get('action')

            if action_type == 'move':
                direction = action.get('direction', 'forward')
                distance = action.get('distance', 1.0)
                self.execute_move_action(direction, distance)

            elif action_type == 'turn':
                direction = action.get('direction', 'left')
                angle = action.get('angle', 90)
                self.execute_turn_action(direction, angle)

            elif action_type == 'navigate':
                location = action.get('location')
                if location:
                    self.execute_navigate_action(location)
                else:
                    self.get_logger().warn('Navigate action missing location parameter')

            elif action_type == 'stop':
                self.execute_stop_action()

            elif action_type == 'follow':
                target = action.get('target', 'person')
                self.execute_follow_action(target)

            elif action_type == 'find_object':
                obj = action.get('object')
                self.execute_find_object_action(obj)

            elif action_type == 'grasp_object':
                obj = action.get('object')
                self.execute_grasp_object_action(obj)

            elif action_type == 'release_object':
                obj = action.get('object')
                self.execute_release_object_action(obj)

            elif action_type == 'describe_environment':
                self.execute_describe_environment_action()

            elif action_type == 'capture_image':
                self.execute_capture_image_action()

            elif action_type == 'provide_help':
                self.execute_provide_help_action()

            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')

            # Brief pause between actions
            time.sleep(0.5)

        # Publish completion status
        status_msg = String()
        status_msg.data = f"COMPLETED_ACTION_SEQUENCE: {len(action_sequence)} actions executed"
        self.status_pub.publish(status_msg)

    def execute_move_action(self, direction: str, distance: float):
        """Execute a move action."""
        cmd = Twist()

        if direction == 'forward':
            cmd.linear.x = 0.3
        elif direction == 'backward':
            cmd.linear.x = -0.3
        elif direction == 'left':
            cmd.linear.y = 0.2
        elif direction == 'right':
            cmd.linear.y = -0.2
        else:
            self.get_logger().warn(f'Unknown move direction: {direction}')
            return

        # Calculate duration based on distance
        duration = distance / 0.3  # Assuming 0.3 m/s speed

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def execute_turn_action(self, direction: str, angle_deg: float):
        """Execute a turn action."""
        cmd = Twist()

        angular_speed = 0.5  # rad/s
        angle_rad = np.deg2rad(angle_deg)
        duration = angle_rad / angular_speed

        if direction == 'left':
            cmd.angular.z = angular_speed
        elif direction == 'right':
            cmd.angular.z = -angular_speed
        else:
            self.get_logger().warn(f'Unknown turn direction: {direction}')
            return

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def execute_navigate_action(self, location: str):
        """Execute a navigation action."""
        if location in self.known_locations:
            location_data = self.known_locations[location]

            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            goal_msg.pose.position.x = location_data['x']
            goal_msg.pose.position.y = location_data['y']
            goal_msg.pose.position.z = 0.0

            # Convert angle to quaternion
            import math
            theta = location_data['theta']
            goal_msg.pose.orientation.z = math.sin(theta / 2.0)
            goal_msg.pose.orientation.w = math.cos(theta / 2.0)

            self.nav_goal_pub.publish(goal_msg)
            self.get_logger().info(f'Navigating to {location} at ({location_data["x"]}, {location_data["y"]})')
        else:
            self.get_logger().warn(f'Unknown location: {location}')

    def execute_stop_action(self):
        """Execute a stop action."""
        self.stop_robot()

    def execute_follow_action(self, target: str):
        """Execute a follow action."""
        self.get_logger().info(f'Following {target}')
        # In a real implementation, this would use person/object detection
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_find_object_action(self, obj: str):
        """Execute a find object action."""
        self.get_logger().info(f'Looking for object: {obj}')
        # In a real implementation, this would use object detection
        # For now, we'll just rotate to scan the environment
        cmd = Twist()
        cmd.angular.z = 0.3
        self.cmd_vel_pub.publish(cmd)
        time.sleep(3.0)  # Rotate for 3 seconds to scan
        self.stop_robot()

    def execute_grasp_object_action(self, obj: str):
        """Execute a grasp object action."""
        self.get_logger().info(f'Attempting to grasp: {obj}')
        # In a real implementation, this would control manipulator arms
        action_msg = String()
        action_msg.data = f"ATTEMPT_GRASP: {obj}"
        self.action_pub.publish(action_msg)

    def execute_release_object_action(self, obj: str):
        """Execute a release object action."""
        self.get_logger().info(f'Releasing: {obj}')
        # In a real implementation, this would control manipulator grippers
        action_msg = String()
        action_msg.data = f"RELEASE_OBJECT: {obj}"
        self.action_pub.publish(action_msg)

    def execute_describe_environment_action(self):
        """Execute a describe environment action."""
        self.get_logger().info('Describing environment')

        description_parts = []

        if self.laser_data:
            obstacles = self.environment_context.get('obstacles', [])
            description_parts.append(f"I see {len(obstacles)} obstacles nearby.")

        if self.environment_context.get('has_camera_data'):
            description_parts.append("I have visual data of the environment.")

        if self.robot_pose:
            description_parts.append(f"My current position is approximately ({self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f}).")

        description = " ".join(description_parts) if description_parts else "I'm unable to perceive the environment properly."

        status_msg = String()
        status_msg.data = f"ENVIRONMENT_DESCRIPTION: {description}"
        self.status_pub.publish(status_msg)

    def execute_capture_image_action(self):
        """Execute a capture image action."""
        self.get_logger().info('Capturing image requested')
        # In a real implementation, this would save the current camera image
        action_msg = String()
        action_msg.data = "IMAGE_CAPTURED"
        self.action_pub.publish(action_msg)

    def execute_provide_help_action(self):
        """Execute a provide help action."""
        help_text = (
            "I can help with navigation, object manipulation, and environmental description. "
            "Try commands like 'go to kitchen', 'find the cup', 'describe the room', or 'take a picture'."
        )

        status_msg = String()
        status_msg.data = f"HELP_INFORMATION: {help_text}"
        self.status_pub.publish(status_msg)

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

    def destroy_node(self):
        """Clean up resources before shutting down."""
        super().destroy_node()
        self.get_logger().info('LLM Integration Node destroyed')


def main(args=None):
    """Main function to run the LLM integration node."""
    rclpy.init(args=args)

    llm_node = LLMIntegrationNode()

    try:
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        llm_node.get_logger().info('LLM integration node stopped by user')
    finally:
        llm_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()