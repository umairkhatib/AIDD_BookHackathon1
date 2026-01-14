#!/usr/bin/env python3

"""
Simple ROS 2 subscriber example for the Physical AI & Humanoid Robotics Course
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()