#!/usr/bin/env python3

"""
Natural Language to Action Sequence Conversion
This module demonstrates converting natural language commands to robot action sequences.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import json
import re
import time
from typing import List, Dict, Any, Optional, Tuple
import numpy as np


class NaturalLanguageToAction(Node):
    """
    A ROS 2 node that converts natural language commands to robot action sequences.
    This demonstrates how LLMs can be used to bridge the gap between human language
    and robot actions.
    """

    def __init__(self):
        super().__init__('natural_language_to_action')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/nl_to_action_status', 10)
        self.action_sequence_pub = self.create_publisher(String, '/action_sequence', 10)

        # Subscribers
        self.nl_command_sub = self.create_subscription(
            String, '/natural_language_command', self.nl_command_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10
        )

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Robot state and environment context
        self.laser_data = None
        self.camera_data = None
        self.robot_pose = None
        self.environment_context = {
            'obstacles': [],
            'free_spaces': [],
            'known_objects': [],
            'robot_capabilities': ['move_forward', 'move_backward', 'turn_left', 'turn_right', 'stop', 'navigate', 'grasp', 'release'],
            'known_locations': ['kitchen', 'living_room', 'bedroom', 'office', 'entrance', 'exit']
        }

        # Command vocabulary and action mappings
        self.command_vocab = {
            # Movement commands
            'move': ['move', 'go', 'travel', 'proceed', 'advance', 'step'],
            'forward': ['forward', 'ahead', 'straight', 'onward'],
            'backward': ['backward', 'back', 'reverse'],
            'left': ['left', 'port', 'starboard_opposite'],
            'right': ['right', 'starboard', 'port_opposite'],
            'turn': ['turn', 'rotate', 'pivot', 'spin'],
            'stop': ['stop', 'halt', 'cease', 'pause', 'freeze'],

            # Navigation commands
            'navigate': ['go to', 'navigate to', 'move to', 'travel to', 'head to', 'proceed to'],
            'location': self.environment_context['known_locations'],

            # Manipulation commands
            'grasp': ['grasp', 'grab', 'pick up', 'take', 'lift', 'hold'],
            'release': ['release', 'drop', 'put down', 'place', 'set down'],
            'manipulate': ['manipulate', 'handle', 'interact with'],

            # Complex commands
            'follow': ['follow', 'accompany', 'accompany', 'escort'],
            'describe': ['describe', 'tell me about', 'what do you see', 'explain'],
            'find': ['find', 'locate', 'search for', 'look for', 'seek'],
            'bring': ['bring', 'fetch', 'get', 'retrieve', 'carry']
        }

        # Action templates
        self.action_templates = {
            'move_forward': {
                'function': self.move_forward,
                'params': ['distance']
            },
            'move_backward': {
                'function': self.move_backward,
                'params': ['distance']
            },
            'turn_left': {
                'function': self.turn_left,
                'params': ['angle']
            },
            'turn_right': {
                'function': self.turn_right,
                'params': ['angle']
            },
            'navigate_to': {
                'function': self.navigate_to,
                'params': ['location']
            },
            'grasp_object': {
                'function': self.grasp_object,
                'params': ['object_name']
            },
            'release_object': {
                'function': self.release_object,
                'params': ['object_name']
            },
            'stop': {
                'function': self.stop_robot,
                'params': []
            },
            'follow': {
                'function': self.follow_target,
                'params': ['target']
            },
            'describe_environment': {
                'function': self.describe_environment,
                'params': []
            },
            'find_object': {
                'function': self.find_object,
                'params': ['object_name']
            }
        }

        # Command parser state
        self.last_parsed_command = None
        self.command_history = []

        self.get_logger().info('Natural Language to Action Converter initialized')

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
            self.environment_context['has_camera_data'] = True
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {e}')

    def nl_command_callback(self, msg):
        """Process natural language command."""
        command_text = msg.data
        self.get_logger().info(f'Received natural language command: "{command_text}"')

        # Parse the command to action sequence
        action_sequence = self.parse_natural_language_to_actions(command_text)

        if action_sequence:
            self.get_logger().info(f'Generated action sequence: {action_sequence}')

            # Publish action sequence for monitoring
            seq_msg = String()
            seq_msg.data = json.dumps(action_sequence)
            self.action_sequence_pub.publish(seq_msg)

            # Execute the action sequence
            self.execute_action_sequence(action_sequence)

            # Update history
            self.command_history.append({
                'input': command_text,
                'actions': action_sequence,
                'timestamp': time.time()
            })

            # Keep only recent history
            if len(self.command_history) > 50:
                self.command_history = self.command_history[-50:]
        else:
            self.get_logger().warn(f'Could not parse command: "{command_text}"')

            # Publish error status
            status_msg = String()
            status_msg.data = f"PARSING_FAILED: {command_text}"
            self.status_pub.publish(status_msg)

    def parse_natural_language_to_actions(self, command_text: str) -> Optional[List[Dict[str, Any]]]:
        """
        Parse natural language command to action sequence.

        This is a rule-based parser that demonstrates how an LLM might process
        natural language commands. In a real implementation, this would use
        a more sophisticated NLP model or LLM API call.
        """
        command_lower = command_text.lower().strip()

        # Store the original command for reference
        self.last_parsed_command = command_text

        # Check for complex commands first
        if self._is_complex_command(command_lower):
            return self._parse_complex_command(command_lower)

        # Simple command parsing
        actions = []

        # Handle multiple commands in sequence (separated by 'and', 'then', etc.)
        if ' and ' in command_lower or ' then ' in command_lower:
            sub_commands = re.split(r' and | then ', command_lower)
            for sub_cmd in sub_commands:
                sub_cmd = sub_cmd.strip()
                if sub_cmd:
                    sub_actions = self._parse_simple_command(sub_cmd)
                    if sub_actions:
                        actions.extend(sub_actions)
            return actions if actions else None

        # Single command
        return self._parse_simple_command(command_lower)

    def _is_complex_command(self, command: str) -> bool:
        """Check if the command is complex (requires planning)."""
        # Complex commands typically contain multiple intents or conditional logic
        complex_indicators = [
            'after', 'when', 'while', 'if', 'until', 'before',
            'first', 'next', 'finally', 'eventually'
        ]

        for indicator in complex_indicators:
            if indicator in command:
                return True
        return False

    def _parse_complex_command(self, command: str) -> Optional[List[Dict[str, Any]]]:
        """Parse complex commands with multiple steps or conditions."""
        command = command.strip()

        # Example: "Go to kitchen and then bring me a cup"
        if ' and ' in command:
            parts = command.split(' and ')
            actions = []
            for part in parts:
                part = part.strip()
                sub_actions = self._parse_simple_command(part)
                if sub_actions:
                    actions.extend(sub_actions)
            return actions if actions else None

        # Example: "Go to kitchen, find a cup, and bring it to me"
        elif ',' in command:
            parts = [p.strip() for p in command.split(',')]
            actions = []
            for part in parts:
                sub_actions = self._parse_simple_command(part)
                if sub_actions:
                    actions.extend(sub_actions)
            return actions if actions else None

        # Example: "Navigate to kitchen then pick up the cup"
        elif ' then ' in command:
            parts = command.split(' then ')
            actions = []
            for part in parts:
                part = part.strip()
                sub_actions = self._parse_simple_command(part)
                if sub_actions:
                    actions.extend(sub_actions)
            return actions if actions else None

        # For other complex patterns, implement additional logic
        return self._parse_simple_command(command)

    def _parse_simple_command(self, command: str) -> Optional[List[Dict[str, Any]]]:
        """Parse a simple command to action(s)."""
        command = command.strip()

        # Movement commands
        if any(move_word in command for move_word in self.command_vocab['move']):
            # Check for forward movement
            if any(forward_word in command for forward_word in self.command_vocab['forward']):
                distance = self._extract_distance(command) or 1.0
                return [{'action': 'move_forward', 'distance': distance}]

            # Check for backward movement
            elif any(backward_word in command for backward_word in self.command_vocab['backward']):
                distance = self._extract_distance(command) or 1.0
                return [{'action': 'move_backward', 'distance': distance}]

            # Check for turning
            elif any(turn_word in command for turn_word in self.command_vocab['turn']):
                if any(left_word in command for left_word in self.command_vocab['left']):
                    angle = self._extract_angle(command) or 90
                    return [{'action': 'turn_left', 'angle': angle}]
                elif any(right_word in command for right_word in self.command_vocab['right']):
                    angle = self._extract_angle(command) or 90
                    return [{'action': 'turn_right', 'angle': angle}]

        # Stop command
        if any(stop_word in command for stop_word in self.command_vocab['stop']):
            return [{'action': 'stop'}]

        # Navigation commands
        for nav_phrase in self.command_vocab['navigate']:
            if nav_phrase in command:
                # Extract location
                location = self._extract_location(command)
                if location:
                    return [{'action': 'navigate_to', 'location': location}]
                else:
                    # If no specific location found, try to extract from known locations
                    for loc in self.environment_context['known_locations']:
                        if loc in command:
                            return [{'action': 'navigate_to', 'location': loc}]

        # Manipulation commands
        if any(grasp_word in command for grasp_word in self.command_vocab['grasp']):
            obj = self._extract_object(command)
            if obj:
                return [
                    {'action': 'find_object', 'object_name': obj},
                    {'action': 'approach_object', 'object_name': obj},
                    {'action': 'grasp_object', 'object_name': obj}
                ]
            else:
                return [{'action': 'find_object', 'object_name': 'unknown'}]

        elif any(release_word in command for release_word in self.command_vocab['release']):
            obj = self._extract_object(command)
            if obj:
                return [{'action': 'release_object', 'object_name': obj}]
            else:
                return [{'action': 'release_object', 'object_name': 'held_item'}]

        elif any(find_word in command for find_word in self.command_vocab['find']):
            obj = self._extract_object(command)
            if obj:
                return [{'action': 'find_object', 'object_name': obj}]
            else:
                return [{'action': 'find_object', 'object_name': 'unknown'}]

        # Complex commands
        if any(bring_word in command for bring_word in self.command_vocab['bring']):
            obj = self._extract_object(command)
            if obj:
                return [
                    {'action': 'find_object', 'object_name': obj},
                    {'action': 'approach_object', 'object_name': obj},
                    {'action': 'grasp_object', 'object_name': obj},
                    {'action': 'navigate_to', 'location': 'user'},  # Simplified user location
                    {'action': 'release_object', 'object_name': obj}
                ]
            else:
                return [{'action': 'find_object', 'object_name': 'unknown'}]

        elif any(describe_word in command for describe_word in self.command_vocab['describe']):
            return [{'action': 'describe_environment'}]

        elif any(follow_word in command for follow_word in self.command_vocab['follow']):
            target = self._extract_target(command) or 'person'
            return [{'action': 'follow', 'target': target}]

        # If no specific action is recognized, try to extract intent
        return self._fallback_intent_extraction(command)

    def _extract_distance(self, command: str) -> Optional[float]:
        """Extract distance from command."""
        # Look for patterns like "2 meters", "3 feet", etc.
        meters_pattern = r'(\d+(?:\.\d+)?)\s*meters?'
        feet_pattern = r'(\d+(?:\.\d+)?)\s*feet|ft'
        meters_match = re.search(meters_pattern, command)
        feet_match = re.search(feet_pattern, command)

        if meters_match:
            return float(meters_match.group(1))
        elif feet_match:
            return float(feet_match.group(1)) * 0.3048  # Convert feet to meters

        # Look for simple numbers near movement words
        numbers = re.findall(r'(\d+(?:\.\d+)?)', command)
        if numbers:
            # Take the first number that seems reasonable for distance
            for num in numbers:
                val = float(num)
                if 0.1 <= val <= 10.0:  # Reasonable distance range
                    return val

        return None

    def _extract_angle(self, command: str) -> Optional[float]:
        """Extract angle from command."""
        degrees_pattern = r'(\d+)\s*degrees?|(\d+)\s*deg'
        numbers = re.findall(r'\d+', command)

        for num in numbers:
            val = int(num)
            if 1 <= val <= 360:  # Valid angle range
                return val

        return None

    def _extract_location(self, command: str) -> Optional[str]:
        """Extract location from command."""
        # Look for known locations in the command
        for location in self.environment_context['known_locations']:
            if location in command:
                return location

        # Could add more sophisticated location extraction here
        # For now, return None if no known location found
        return None

    def _extract_object(self, command: str) -> Optional[str]:
        """Extract object from command."""
        # Common objects that might be in the environment
        common_objects = [
            'cup', 'bottle', 'box', 'chair', 'table', 'book',
            'pen', 'phone', 'keys', 'wallet', 'apple', 'banana',
            'cup', 'plate', 'fork', 'spoon', 'bowl', 'glass'
        ]

        for obj in common_objects:
            if obj in command:
                return obj

        # Extract any noun that might be an object
        # This is a simplified approach - in reality, you'd use NLP
        words = command.split()
        for word in words:
            # Look for words that might be objects (not verbs, adjectives, etc.)
            if len(word) > 3 and word.isalpha():  # Basic filter
                # Check if it sounds like an object
                if word.endswith(('er', 'or', 'tion', 'ing', 'ed', 'ly')):
                    continue  # Likely not an object
                return word

        return None

    def _extract_target(self, command: str) -> Optional[str]:
        """Extract target from command."""
        if 'me' in command or 'my' in command:
            return 'user'
        elif 'person' in command:
            return 'person'
        elif 'object' in command:
            return 'object'
        else:
            return 'closest_target'

    def _fallback_intent_extraction(self, command: str) -> Optional[List[Dict[str, Any]]]:
        """Fallback intent extraction for unrecognized commands."""
        command_lower = command.lower()

        # Try to identify general categories
        if any(word in command_lower for word in ['move', 'go', 'travel']):
            return [{'action': 'move_forward', 'distance': 1.0}]

        elif any(word in command_lower for word in ['stop', 'halt', 'wait']):
            return [{'action': 'stop'}]

        elif any(word in command_lower for word in ['turn', 'rotate']):
            return [{'action': 'turn_left', 'angle': 90}]

        elif any(word in command_lower for word in ['help', 'assist']):
            return [{'action': 'provide_help'}]

        else:
            # Command not understood
            self.get_logger().warn(f'Command not understood: {command}')
            return None

    def execute_action_sequence(self, action_sequence: List[Dict[str, Any]]):
        """Execute a sequence of actions."""
        self.get_logger().info(f'Executing action sequence: {action_sequence}')

        for action in action_sequence:
            action_type = action.get('action')

            if action_type in self.action_templates:
                func = self.action_templates[action_type]['function']
                params = {k: v for k, v in action.items() if k != 'action'}

                try:
                    self.get_logger().info(f'Executing action: {action_type} with params: {params}')
                    func(**params)

                    # Brief pause between actions
                    time.sleep(0.5)
                except Exception as e:
                    self.get_logger().error(f'Error executing action {action_type}: {e}')
                    status_msg = String()
                    status_msg.data = f"ACTION_ERROR: {action_type} - {str(e)}"
                    self.status_pub.publish(status_msg)
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                status_msg = String()
                status_msg.data = f"UNKNOWN_ACTION: {action_type}"
                self.status_pub.publish(status_msg)

        # Publish completion status
        status_msg = String()
        status_msg.data = f"SEQUENCE_COMPLETED: {len(action_sequence)} actions executed"
        self.status_pub.publish(status_msg)

    def move_forward(self, distance: float = 1.0):
        """Move robot forward."""
        cmd = Twist()
        cmd.linear.x = 0.3  # Fixed speed
        duration = distance / 0.3  # Time needed to travel the distance

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def move_backward(self, distance: float = 1.0):
        """Move robot backward."""
        cmd = Twist()
        cmd.linear.x = -0.3  # Fixed speed
        duration = distance / 0.3  # Time needed to travel the distance

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def turn_left(self, angle: float = 90.0):
        """Turn robot left."""
        cmd = Twist()
        angular_speed = 0.5  # rad/s
        angle_rad = np.deg2rad(angle)
        duration = angle_rad / angular_speed

        cmd.angular.z = angular_speed
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def turn_right(self, angle: float = 90.0):
        """Turn robot right."""
        cmd = Twist()
        angular_speed = 0.5  # rad/s
        angle_rad = np.deg2rad(angle)
        duration = angle_rad / angular_speed

        cmd.angular.z = -angular_speed  # Negative for right turn
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # Stop robot
        self.stop_robot()

    def navigate_to(self, location: str):
        """Navigate to a specific location."""
        if location in self.known_locations:
            location_data = self.known_locations[location]

            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            goal_msg.pose.position.x = location_data['x']
            goal_msg.pose.position.y = location_data['y']
            goal_msg.pose.position.z = 0.0

            # Convert theta to quaternion
            import math
            theta = location_data['theta']
            goal_msg.pose.orientation.z = math.sin(theta / 2.0)
            goal_msg.pose.orientation.w = math.cos(theta / 2.0)

            self.nav_goal_pub.publish(goal_msg)
            self.get_logger().info(f'Navigating to {location} at ({location_data["x"]}, {location_data["y"]})')
        else:
            self.get_logger().warn(f'Unknown location: {location}')
            status_msg = String()
            status_msg.data = f"UNKNOWN_LOCATION: {location}"
            self.status_pub.publish(status_msg)

    def grasp_object(self, object_name: str):
        """Grasp an object."""
        self.get_logger().info(f'Attempting to grasp {object_name}')

        # In a real implementation, this would interface with a manipulator
        # For this example, we'll just log the action
        action_msg = String()
        action_msg.data = f"ATTEMPT_GRASP: {object_name}"
        self.action_pub.publish(action_msg)

    def release_object(self, object_name: str):
        """Release an object."""
        self.get_logger().info(f'Releasing {object_name}')

        # In a real implementation, this would interface with a manipulator
        # For this example, we'll just log the action
        action_msg = String()
        action_msg.data = f"RELEASE_OBJECT: {object_name}"
        self.action_pub.publish(action_msg)

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

    def follow_target(self, target: str = 'person'):
        """Follow a target."""
        self.get_logger().info(f'Following {target}')

        # In a real implementation, this would use person/object detection
        # For this example, we'll just move forward slowly
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def describe_environment(self):
        """Describe the environment."""
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

    def find_object(self, object_name: str):
        """Find an object in the environment."""
        self.get_logger().info(f'Looking for object: {object_name}')

        # In a real implementation, this would use object detection
        # For this example, we'll just rotate to scan the environment
        cmd = Twist()
        cmd.angular.z = 0.3  # Rotate slowly
        self.cmd_vel_pub.publish(cmd)
        time.sleep(3.0)  # Rotate for 3 seconds to scan
        self.stop_robot()

        # Publish status
        status_msg = String()
        status_msg.data = f"SEARCH_COMPLETE: {object_name}"
        self.status_pub.publish(status_msg)

    def provide_help(self):
        """Provide help information."""
        help_text = (
            "I can understand commands like: "
            "'move forward 2 meters', "
            "'turn left 90 degrees', "
            "'go to kitchen', "
            "'pick up the cup', "
            "'find the book', "
            "'stop', "
            "'describe the room', "
            "'follow me', "
            "'bring me coffee'"
        )

        status_msg = String()
        status_msg.data = f"HELP_AVAILABLE: {help_text}"
        self.status_pub.publish(status_msg)

    def get_command_history(self) -> List[Dict[str, Any]]:
        """Get the command history."""
        return self.command_history

    def destroy_node(self):
        """Clean up resources before shutting down."""
        # Stop robot before shutting down
        self.stop_robot()
        super().destroy_node()
        self.get_logger().info('Natural Language to Action node destroyed')


def main(args=None):
    """Main function to run the NL to Action converter."""
    rclpy.init(args=args)

    nl_to_action = NaturalLanguageToAction()

    try:
        rclpy.spin(nl_to_action)
    except KeyboardInterrupt:
        nl_to_action.get_logger().info('NL to Action converter stopped by user')
    finally:
        nl_to_action.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()