#!/usr/bin/env python3

"""
Hello World Example for Physical AI & Humanoid Robotics Course
This script verifies that the development environment is set up correctly.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldPublisher(Node):

    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_world', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    hello_world_publisher = HelloWorldPublisher()

    print("Hello World Publisher is running...")
    print("Expected output: Messages published to 'hello_world' topic")
    print("Press Ctrl+C to stop")

    try:
        rclpy.spin(hello_world_publisher)
    except KeyboardInterrupt:
        print("\nShutting down...")

    # Destroy the node explicitly
    hello_world_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()