#!/usr/bin/env python3

"""
Sensor Simulation Example for the Physical AI & Humanoid Robotics Course
This example demonstrates how to simulate various robot sensors in ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, MagneticField, NavSatFix, PointCloud2, PointField
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import math
import numpy as np
from tf2_ros import TransformBroadcaster
import random


class SensorSimulator(Node):
    """
    Sensor simulator node that demonstrates various sensor simulations.
    """

    def __init__(self):
        super().__init__('sensor_simulator')

        # Publishers for different sensor types
        self.laser_pub = self.create_publisher(LaserScan, '/laser_scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # Timer for sensor data publishing
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)  # 10Hz

        # Robot state for simulation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_vx = 0.5  # Constant velocity for simulation
        self.robot_vy = 0.0
        self.robot_vtheta = 0.0

        self.get_logger().info('Sensor Simulator initialized')

    def generate_laser_scan(self):
        """Generate simulated laser scan data."""
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Laser scan parameters
        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2   # 90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Calculate number of ranges
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = []

        # Simulate some obstacles around the robot
        for i in range(num_ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Simulate a wall in front of the robot
            distance_to_wall = 3.0  # Wall is 3 meters away

            # Add some noise to the distance
            noisy_distance = distance_to_wall + random.uniform(-0.1, 0.1)

            # Add some variation based on angle to simulate corners
            if -0.5 < angle < 0.5:  # Front of robot
                range_val = max(msg.range_min, min(noisy_distance, msg.range_max))
            elif angle < -0.7 or angle > 0.7:  # Sides of robot
                range_val = max(msg.range_min, min(distance_to_wall + 1.0, msg.range_max))
            else:
                range_val = msg.range_max  # Far distances

            msg.ranges.append(range_val)

        return msg

    def generate_imu_data(self):
        """Generate simulated IMU data."""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_frame'

        # Simulate orientation (for this example, robot is upright)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.robot_theta / 2)
        msg.orientation.w = math.cos(self.robot_theta / 2)

        # Angular velocity (robot turning)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = self.robot_vtheta + random.uniform(-0.01, 0.01)

        # Linear acceleration (including gravity effect)
        msg.linear_acceleration.x = self.robot_vx * 0.1 + random.uniform(-0.1, 0.1)  # Forward acceleration
        msg.linear_acceleration.y = self.robot_vy * 0.1 + random.uniform(-0.1, 0.1)  # Side acceleration
        msg.linear_acceleration.z = 9.81 + random.uniform(-0.1, 0.1)  # Gravity

        # Covariances (set to non-zero values as per REP-145)
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        return msg

    def generate_magnetic_field(self):
        """Generate simulated magnetic field data."""
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mag_frame'

        # Earth's magnetic field (approximate)
        msg.magnetic_field.x = 0.22  # Tesla
        msg.magnetic_field.y = 0.0
        msg.magnetic_field.z = 0.45

        # Add some noise
        msg.magnetic_field.x += random.uniform(-0.01, 0.01)
        msg.magnetic_field.y += random.uniform(-0.01, 0.01)
        msg.magnetic_field.z += random.uniform(-0.01, 0.01)

        # Magnetic field covariance
        msg.magnetic_field_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        return msg

    def generate_gps_data(self):
        """Generate simulated GPS data."""
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_frame'

        # Simulate GPS coordinates near a typical location
        msg.latitude = 37.7749 + self.robot_x * 0.00001  # Roughly 1 meter per unit
        msg.longitude = -122.4194 + self.robot_y * 0.00001  # Roughly 1 meter per unit
        msg.altitude = 10.0 + random.uniform(-0.5, 0.5)  # Altitude with noise

        # Position covariance (accurate GPS)
        msg.position_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.2
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        # Status
        msg.status.status = NavSatFix.STATUS_FIX
        msg.status.service = NavSatFix.CONSTELLATION_GPS_SERVICE

        return msg

    def update_robot_state(self):
        """Update robot state for simulation."""
        # Simple motion model
        dt = 0.1  # Time step (from timer)
        self.robot_x += self.robot_vx * dt * math.cos(self.robot_theta) - self.robot_vy * dt * math.sin(self.robot_theta)
        self.robot_y += self.robot_vx * dt * math.sin(self.robot_theta) + self.robot_vy * dt * math.cos(self.robot_theta)
        self.robot_theta += self.robot_vtheta * dt

        # Keep theta in [-pi, pi]
        if self.robot_theta > math.pi:
            self.robot_theta -= 2 * math.pi
        elif self.robot_theta < -math.pi:
            self.robot_theta += 2 * math.pi

    def publish_sensor_data(self):
        """Publish all sensor data."""
        # Update robot state
        self.update_robot_state()

        # Publish laser scan
        laser_msg = self.generate_laser_scan()
        self.laser_pub.publish(laser_msg)

        # Publish IMU data
        imu_msg = self.generate_imu_data()
        self.imu_pub.publish(imu_msg)

        # Publish magnetic field data
        mag_msg = self.generate_magnetic_field()
        self.mag_pub.publish(mag_msg)

        # Publish GPS data
        gps_msg = self.generate_gps_data()
        self.gps_pub.publish(gps_msg)

        # Log position
        self.get_logger().info(f'Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}), theta: {self.robot_theta:.2f}', throttle_duration_sec=1)


def main(args=None):
    """Main function to run the sensor simulator."""
    rclpy.init(args=args)

    sensor_simulator = SensorSimulator()

    try:
        rclpy.spin(sensor_simulator)
    except KeyboardInterrupt:
        sensor_simulator.get_logger().info('Sensor simulator stopped by user')
    finally:
        sensor_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()