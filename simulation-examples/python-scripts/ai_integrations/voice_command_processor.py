#!/usr/bin/env python3

"""
Voice Command Processor for Physical AI & Humanoid Robotics Course
This module implements voice command processing using Whisper for the VLA system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import whisper
import pyaudio
import numpy as np
import wave
import tempfile
import os
import threading
import queue
import time
from typing import Dict, List, Optional, Any
import json


class VoiceCommandProcessor(Node):
    """
    A ROS 2 node that processes voice commands using Whisper and converts them to robot actions.
    """

    def __init__(self):
        super().__init__('voice_command_processor')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)
        self.action_pub = self.create_publisher(String, '/robot_action', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10
        )

        # Initialize Whisper model
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("base")  # Use "tiny" for faster processing if needed
        self.get_logger().info('Whisper model loaded successfully')

        # Audio parameters
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.silence_threshold = 500
        self.record_seconds = 3  # Record 3 seconds of audio when voice detected

        # Initialize audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Audio processing queue
        self.audio_queue = queue.Queue()
        self.processing_active = True

        # Robot state and context
        self.laser_data = None
        self.camera_data = None
        self.robot_pose = None
        self.environment_context = {
            'obstacles': [],
            'free_spaces': [],
            'last_command_time': time.time()
        }

        # CV Bridge for image processing
        self.cv_bridge = CvBridge()

        # Command mapping
        self.command_map = {
            'move forward': self.move_forward,
            'go forward': self.move_forward,
            'move backward': self.move_backward,
            'go back': self.move_backward,
            'move left': self.move_left,
            'turn left': self.turn_left,
            'move right': self.move_right,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'halt': self.stop_robot,
            'spin left': self.spin_left,
            'spin right': self.spin_right,
            'go to kitchen': lambda: self.navigate_to_location('kitchen'),
            'go to living room': lambda: self.navigate_to_location('living_room'),
            'go to bedroom': lambda: self.navigate_to_location('bedroom'),
            'go to office': lambda: self.navigate_to_location('office'),
            'follow me': self.follow_person,
            'come here': self.come_here,
            'take picture': self.take_picture,
            'what do you see': self.describe_scene
        }

        # Known locations with coordinates (these would be defined based on your map)
        self.known_locations = {
            'kitchen': {'x': 5.0, 'y': 3.0, 'theta': 0.0},
            'living_room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'bedroom': {'x': -3.0, 'y': 2.0, 'theta': 1.57},
            'office': {'x': 2.0, 'y': -2.0, 'theta': 3.14}
        }

        # Start audio processing thread
        self.audio_thread = threading.Thread(target=self.audio_processing_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()

        # Timer for status updates
        self.status_timer = self.create_timer(5.0, self.publish_status)

        self.get_logger().info('Voice Command Processor initialized and ready')

    def scan_callback(self, msg):
        """Process LIDAR data for obstacle detection."""
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
        """Process camera data."""
        try:
            self.camera_data = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {e}')

    def audio_processing_loop(self):
        """Continuously process audio for voice commands."""
        while self.processing_active and rclpy.ok():
            try:
                # Read audio chunk
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                audio_array = np.frombuffer(data, dtype=np.int16)

                # Check for voice activity
                if np.abs(audio_array).mean() > self.silence_threshold:
                    self.get_logger().info('Voice activity detected, recording...')

                    # Record a longer segment
                    frames = [data]
                    for _ in range(0, int(self.rate / self.chunk * self.record_seconds)):
                        data = self.stream.read(self.chunk, exception_on_overflow=False)
                        frames.append(data)

                    # Convert to WAV format and save temporarily
                    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                        wf = wave.open(temp_file.name, 'wb')
                        wf.setnchannels(self.channels)
                        wf.setsampwidth(self.audio.get_sample_size(self.audio_format))
                        wf.setframerate(self.rate)
                        wf.writeframes(b''.join(frames))
                        wf.close()

                        # Transcribe using Whisper
                        result = self.model.transcribe(temp_file.name)
                        transcribed_text = result["text"].strip()

                        if transcribed_text:
                            self.get_logger().info(f'Whisper transcribed: "{transcribed_text}"')

                            # Process the command in the main thread
                            self.process_voice_command(transcribed_text)

                        # Clean up temporary file
                        os.unlink(temp_file.name)

                time.sleep(0.1)  # Brief pause to prevent excessive CPU usage

            except Exception as e:
                self.get_logger().error(f'Error in audio processing: {e}')
                time.sleep(0.1)  # Brief pause before continuing

    def process_voice_command(self, command_text: str):
        """Process the transcribed voice command."""
        # Update last command time
        self.environment_context['last_command_time'] = time.time()

        # Publish status
        status_msg = String()
        status_msg.data = f"RECEIVED_COMMAND: {command_text}"
        self.status_pub.publish(status_msg)

        # Convert to lowercase for matching
        command_lower = command_text.lower()

        # Check for command matches
        matched = False
        for cmd_phrase, cmd_func in self.command_map.items():
            if cmd_phrase in command_lower:
                self.get_logger().info(f'Matched command: "{cmd_phrase}", executing...')

                try:
                    cmd_func()

                    # Publish action confirmation
                    action_msg = String()
                    action_msg.data = f"EXECUTED: {cmd_phrase}"
                    self.action_pub.publish(action_msg)

                    matched = True
                    break
                except Exception as e:
                    self.get_logger().error(f'Error executing command "{cmd_phrase}": {e}')
                    error_msg = String()
                    error_msg.data = f"ERROR_EXECUTING: {cmd_phrase} - {str(e)}"
                    self.status_pub.publish(error_msg)

        if not matched:
            # Try to interpret as a navigation command with location
            self.attempt_navigation_by_location(command_lower)

        if not matched:
            self.get_logger().warn(f'Unrecognized command: "{command_text}"')
            error_msg = String()
            error_msg.data = f"UNRECOGNIZED_COMMAND: {command_text}"
            self.status_pub.publish(error_msg)

    def attempt_navigation_by_location(self, command_lower: str):
        """Attempt to extract location from command for navigation."""
        for location_name in self.known_locations:
            if location_name in command_lower:
                self.get_logger().info(f'Navigating to extracted location: {location_name}')
                self.navigate_to_location(location_name)
                return True
        return False

    def move_forward(self):
        """Move robot forward."""
        cmd = Twist()
        cmd.linear.x = 0.3  # Moderate speed
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Moving forward')

    def move_backward(self):
        """Move robot backward."""
        cmd = Twist()
        cmd.linear.x = -0.3
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Moving backward')

    def move_left(self):
        """Move robot to the left."""
        cmd = Twist()
        cmd.linear.y = 0.2  # Sideways movement
        cmd.linear.x = 0.1  # Forward component to maintain momentum
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Moving left')

    def move_right(self):
        """Move robot to the right."""
        cmd = Twist()
        cmd.linear.y = -0.2  # Sideways movement
        cmd.linear.x = 0.1  # Forward component to maintain momentum
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Moving right')

    def turn_left(self):
        """Turn robot left."""
        cmd = Twist()
        cmd.linear.x = 0.1  # Slow forward movement while turning
        cmd.angular.z = 0.5  # Positive for counter-clockwise
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Turning left')

    def turn_right(self):
        """Turn robot right."""
        cmd = Twist()
        cmd.linear.x = 0.1  # Slow forward movement while turning
        cmd.angular.z = -0.5  # Negative for clockwise
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Turning right')

    def spin_left(self):
        """Spin robot in place to the left."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.8
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Spinning left')

    def spin_right(self):
        """Spin robot in place to the right."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.8
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Spinning right')

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

    def navigate_to_location(self, location_name: str):
        """Navigate to a known location."""
        if location_name in self.known_locations:
            location_data = self.known_locations[location_name]

            # Check if path is clear using LIDAR data
            obstacles = self.environment_context.get('obstacles', [])

            # For this example, we'll just send a basic navigation goal
            # In a real implementation, you would check if the path is clear
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

            self.get_logger().info(f'Navigating to {location_name} at ({location_data["x"]}, {location_data["y"]})')
        else:
            self.get_logger().warn(f'Unknown location: {location_name}')

    def follow_person(self):
        """Initiate person following behavior."""
        self.get_logger().info('Initiating person following behavior')

        # In a real implementation, this would use person detection and tracking
        # For this example, we'll just move forward slowly
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def come_here(self):
        """Come to the user's location."""
        self.get_logger().info('Coming to user location')

        # In a real implementation, this would require localization of the user
        # For this example, we'll just move forward
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def take_picture(self):
        """Take a picture with the robot's camera."""
        self.get_logger().info('Taking picture requested')

        # In a real implementation, this would capture and save an image
        # For this example, we'll just log the action
        action_msg = String()
        action_msg.data = "PICTURE_TAKEN"
        self.action_pub.publish(action_msg)

    def describe_scene(self):
        """Describe what the robot sees."""
        self.get_logger().info('Describing scene requested')

        # In a real implementation, this would use computer vision to describe the scene
        # For this example, we'll just report what we know about the environment
        if self.laser_data:
            obstacles = self.environment_context.get('obstacles', [])
            description = f"I see {len(obstacles)} obstacles nearby."

            status_msg = String()
            status_msg.data = f"SCENE_DESCRIPTION: {description}"
            self.status_pub.publish(status_msg)
        else:
            status_msg = String()
            status_msg.data = "SCENE_DESCRIPTION: Unable to see environment - no sensor data"
            self.status_pub.publish(status_msg)

    def publish_status(self):
        """Publish periodic status updates."""
        status_msg = String()
        status_msg.data = f"VOICE_PROCESSOR_STATUS:active|obstacles={len(self.environment_context['obstacles'])}|last_cmd={self.environment_context['last_command_time']:.0f}"
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean up resources before shutting down."""
        self.processing_active = False

        # Close audio stream
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

        super().destroy_node()
        self.get_logger().info('Voice Command Processor node destroyed')


def main(args=None):
    """Main function to run the voice command processor."""
    rclpy.init(args=args)

    voice_processor = VoiceCommandProcessor()

    try:
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        voice_processor.get_logger().info('Voice command processor stopped by user')
    finally:
        voice_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()