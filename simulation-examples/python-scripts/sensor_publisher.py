#!/usr/bin/env python3

"""
Sensor Data Publisher for the Physical AI & Humanoid Robotics Course
This module provides utilities for publishing various types of sensor data in ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import (
    LaserScan, Imu, MagneticField, NavSatFix, PointCloud2, PointField,
    Joy, Temperature, FluidPressure, RelativeHumidity
)
from geometry_msgs.msg import Vector3Stamped, TwistWithCovarianceStamped
from std_msgs.msg import Header, Float64
from builtin_interfaces.msg import Time
import math
import numpy as np
import random
from typing import List, Tuple, Optional


class SensorPublisher(Node):
    """
    A comprehensive sensor publisher that can simulate various robot sensors.
    """

    def __init__(self):
        super().__init__('sensor_publisher')

        # Publishers for different sensor types
        self.laser_pub = self.create_publisher(LaserScan, '/laser_scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.temp_pub = self.create_publisher(Temperature, '/temperature', 10)
        self.pressure_pub = self.create_publisher(FluidPressure, '/pressure', 10)
        self.humidity_pub = self.create_publisher(RelativeHumidity, '/humidity', 10)

        # Timer for sensor data publishing
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)  # 10Hz

        # Robot state for simulation
        self.time_offset = self.get_clock().now().seconds_nanoseconds()[0]
        self.simulation_time = 0.0

        # Robot pose and motion parameters
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_vx = 0.5  # Constant velocity for simulation
        self.robot_vy = 0.0
        self.robot_vtheta = 0.0

        self.get_logger().info('Sensor Publisher initialized')

    def create_header(self, frame_id: str = 'sensor_frame') -> Header:
        """Create a header with timestamp."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def generate_laser_scan(self) -> LaserScan:
        """Generate simulated laser scan data."""
        msg = LaserScan()
        msg.header = self.create_header('laser_frame')

        # Laser scan parameters
        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2   # 90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree increment
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

    def generate_imu_data(self) -> Imu:
        """Generate simulated IMU data."""
        msg = Imu()
        msg.header = self.create_header('imu_frame')

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

    def generate_magnetic_field(self) -> MagneticField:
        """Generate simulated magnetic field data."""
        msg = MagneticField()
        msg.header = self.create_header('mag_frame')

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

    def generate_gps_data(self) -> NavSatFix:
        """Generate simulated GPS data."""
        msg = NavSatFix()
        msg.header = self.create_header('gps_frame')

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

    def generate_temperature_data(self) -> Temperature:
        """Generate simulated temperature data."""
        msg = Temperature()
        msg.header = self.create_header('temp_frame')

        # Simulate ambient temperature with slight variations
        base_temp = 22.0  # 22°C room temperature
        msg.temperature = base_temp + random.uniform(-1.0, 1.0)

        # Variance (uncertainty)
        msg.variance = 0.1

        return msg

    def generate_pressure_data(self) -> FluidPressure:
        """Generate simulated pressure data."""
        msg = FluidPressure()
        msg.header = self.create_header('pressure_frame')

        # Simulate atmospheric pressure (about 101325 Pa at sea level)
        base_pressure = 101325.0
        # Add altitude effect and noise
        altitude_effect = -self.robot_y * 12  # Rough approximation: 12 Pa per meter
        msg.fluid_pressure = base_pressure + altitude_effect + random.uniform(-50, 50)

        # Variance (uncertainty)
        msg.variance = 100.0

        return msg

    def generate_humidity_data(self) -> RelativeHumidity:
        """Generate simulated humidity data."""
        msg = RelativeHumidity()
        msg.header = self.create_header('humidity_frame')

        # Simulate relative humidity
        base_humidity = 50.0  # 50% humidity
        msg.relative_humidity = base_humidity + random.uniform(-5.0, 5.0)

        # Variance (uncertainty)
        msg.variance = 1.0

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

        # Publish all sensor data
        self.laser_pub.publish(self.generate_laser_scan())
        self.imu_pub.publish(self.generate_imu_data())
        self.mag_pub.publish(self.generate_magnetic_field())
        self.gps_pub.publish(self.generate_gps_data())
        self.temp_pub.publish(self.generate_temperature_data())
        self.pressure_pub.publish(self.generate_pressure_data())
        self.humidity_pub.publish(self.generate_humidity_data())

        # Log position periodically
        if int(self.get_clock().now().seconds_nanoseconds()[0]) % 5 == 0:
            self.get_logger().info(f'Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}), theta: {self.robot_theta:.2f}')


class SensorManager:
    """
    A higher-level sensor manager that can coordinate multiple sensor publishers.
    """

    def __init__(self, node: Node):
        self.node = node
        self.sensors = {}

    def add_sensor_publisher(self, name: str, publisher):
        """Add a sensor publisher to the manager."""
        self.sensors[name] = publisher

    def get_sensor_data(self, sensor_name: str):
        """Get data from a specific sensor."""
        if sensor_name in self.sensors:
            return self.sensors[sensor_name]
        return None

    def sync_sensor_data(self):
        """Synchronize sensor data to maintain temporal consistency."""
        current_time = self.node.get_clock().now()
        # This would be used to align timestamps of different sensors
        pass


def main(args=None):
    """Main function to run the sensor publisher."""
    rclpy.init(args=args)

    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        sensor_publisher.get_logger().info('Sensor publisher stopped by user')
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()