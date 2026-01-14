#!/usr/bin/env python3

"""
Localization Examples for the Physical AI & Humanoid Robotics Course
This module demonstrates various localization techniques and algorithms.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Float64
from tf2_ros import TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import PoseStamped as TfPoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple, Dict, Any
import time
import random


class LocalizationNode(Node):
    """
    A node that demonstrates various localization techniques including:
    - Particle Filter (Monte Carlo Localization)
    - Extended Kalman Filter (EKF)
    - AMCL (Adaptive Monte Carlo Localization)
    - Visual SLAM
    """

    def __init__(self):
        super().__init__('localization_node')

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pose_estimate', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(String, '/localization_status', 10)
        self.error_pub = self.create_publisher(Float64, '/localization_error', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth/odom', self.ground_truth_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Localization state
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.ground_truth_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.localization_method = 'particle_filter'  # Default method
        self.map_data = None
        self.laser_data = None
        self.imu_data = None

        # Particle filter parameters
        self.num_particles = 1000
        self.particles = np.zeros((self.num_particles, 3))  # x, y, theta
        self.weights = np.ones(self.num_particles) / self.num_particles

        # Motion model parameters
        self.motion_noise = {'linear': 0.1, 'angular': 0.05}
        self.measurement_noise = 0.1

        # Timer for localization processing
        self.loc_timer = self.create_timer(0.1, self.localization_loop)

        # Initialize particles
        self.initialize_particles()

        self.get_logger().info('Localization Node initialized')

    def initialize_particles(self):
        """Initialize particles for particle filter."""
        # For demonstration, initialize around (0, 0) with some spread
        self.particles[:, 0] = np.random.normal(0, 1, self.num_particles)  # x
        self.particles[:, 1] = np.random.normal(0, 1, self.num_particles)  # y
        self.particles[:, 2] = np.random.uniform(-np.pi, np.pi, self.num_particles)  # theta

        self.get_logger().info(f'Initialized {self.num_particles} particles')

    def scan_callback(self, msg):
        """Process laser scan data."""
        self.laser_data = msg
        # In a real implementation, this would be used for likelihood field matching
        # or beam model evaluation in particle filter

    def imu_callback(self, msg):
        """Process IMU data."""
        self.imu_data = msg
        # In a real implementation, this would provide orientation data

    def ground_truth_callback(self, msg):
        """Receive ground truth pose for comparison."""
        self.ground_truth_pose['x'] = msg.pose.pose.position.x
        self.ground_truth_pose['y'] = msg.pose.pose.position.y

        # Convert quaternion to euler
        orientation_q = msg.pose.pose.orientation
        _, _, self.ground_truth_pose['theta'] = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def map_callback(self, msg):
        """Process map data."""
        self.map_data = msg
        # Store map information for localization algorithms

    def motion_model(self, prev_pose, control, dt=0.1):
        """
        Simple motion model for robot movement prediction.
        control: [linear_velocity, angular_velocity]
        """
        linear_v, angular_v = control

        # Add noise to motion
        linear_noise = np.random.normal(0, self.motion_noise['linear'])
        angular_noise = np.random.normal(0, self.motion_noise['angular'])

        # Update pose based on motion model
        new_theta = prev_pose[2] + (angular_v + angular_noise) * dt
        new_x = prev_pose[0] + (linear_v + linear_noise) * math.cos(prev_pose[2]) * dt
        new_y = prev_pose[1] + (linear_v + linear_noise) * math.sin(prev_pose[2]) * dt

        return [new_x, new_y, new_theta]

    def measurement_model(self, particle_pose, scan_data):
        """
        Measurement model for particle weight calculation.
        This is a simplified version - in real implementation, this would use
        likelihood field matching or beam model.
        """
        if scan_data is None:
            return 1.0  # Neutral weight if no scan data

        # Simplified measurement model - just return a weight based on some arbitrary metric
        # In real implementation, this would compare predicted measurements with actual measurements
        expected_range = 2.0  # Expected range for demonstration
        actual_range = min(scan_data.ranges) if scan_data.ranges else expected_range

        # Calculate weight based on difference
        weight = math.exp(-abs(actual_range - expected_range) / self.measurement_noise)
        return max(weight, 0.01)  # Prevent zero weights

    def resample_particles(self):
        """Resample particles based on their weights."""
        # Normalize weights
        self.weights /= np.sum(self.weights)

        # Systematic resampling
        indices = []
        cumulative_sum = np.cumsum(self.weights)
        j = 0
        U = np.random.uniform(0, 1.0/self.num_particles)

        for i in range(self.num_particles):
            while U > cumulative_sum[j]:
                j = j + 1
            indices.append(j)
            U += 1.0/self.num_particles

        # Update particles
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def particle_filter_localization(self, control):
        """Execute particle filter localization."""
        # Predict step: move particles according to motion model
        for i in range(self.num_particles):
            self.particles[i] = self.motion_model(self.particles[i], control)

        # Update step: evaluate particles based on sensor data
        if self.laser_data:
            for i in range(self.num_particles):
                weight = self.measurement_model(self.particles[i], self.laser_data)
                self.weights[i] *= weight

        # Normalize weights
        self.weights = np.maximum(self.weights, 1e-300)  # Prevent underflow
        self.weights /= np.sum(self.weights)

        # Resample if effective number of particles is low
        neff = 1.0 / np.sum(self.weights**2)
        if neff < self.num_particles / 2.0:
            self.resample_particles()

        # Estimate pose as weighted average of particles
        estimated_pose = np.average(self.particles, axis=0, weights=self.weights)
        return estimated_pose

    def ekf_localization(self):
        """Execute Extended Kalman Filter localization (simplified)."""
        # This is a simplified version - in real EKF, you would maintain state and covariance
        # matrices and perform prediction and update steps with Jacobians
        estimated_pose = [
            self.current_pose['x'],
            self.current_pose['y'],
            self.current_pose['theta']
        ]
        return estimated_pose

    def amcl_localization(self, control):
        """Execute AMCL (Adaptive Monte Carlo Localization)."""
        # AMCL adapts the number of particles based on uncertainty
        # This is a simplified version of the actual AMCL algorithm
        estimated_pose = self.particle_filter_localization(control)

        # Adapt number of particles based on uncertainty
        uncertainty = np.std(self.particles, axis=0)
        max_uncertainty = np.max(uncertainty)

        if max_uncertainty > 0.5:  # High uncertainty threshold
            # Increase particles temporarily
            self.get_logger().info('High uncertainty detected, increasing particles')
            # In real implementation, this would dynamically adjust particle count

        return estimated_pose

    def calculate_localization_error(self, estimated_pose):
        """Calculate localization error compared to ground truth."""
        dx = estimated_pose[0] - self.ground_truth_pose['x']
        dy = estimated_pose[1] - self.ground_truth_pose['y']
        dtheta = estimated_pose[2] - self.ground_truth_pose['theta']

        # Normalize angle difference
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi

        error = math.sqrt(dx*dx + dy*dy)
        return error

    def localization_loop(self):
        """Main localization processing loop."""
        # Simulate control input (this would come from motion commands in real system)
        control_input = [0.2, 0.05]  # [linear_vel, angular_vel]

        # Execute localization based on selected method
        if self.localization_method == 'particle_filter':
            estimated_pose = self.particle_filter_localization(control_input)
        elif self.localization_method == 'ekf':
            estimated_pose = self.ekf_localization()
        elif self.localization_method == 'amcl':
            estimated_pose = self.amcl_localization(control_input)
        else:
            # Default to particle filter
            estimated_pose = self.particle_filter_localization(control_input)

        # Update current pose estimate
        self.current_pose['x'] = estimated_pose[0]
        self.current_pose['y'] = estimated_pose[1]
        self.current_pose['theta'] = estimated_pose[2]

        # Publish pose estimate
        self.publish_pose_estimate()

        # Calculate and publish localization error
        error = self.calculate_localization_error(estimated_pose)
        error_msg = Float64()
        error_msg.data = error
        self.error_pub.publish(error_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f"LOCALIZED:x={estimated_pose[0]:.2f},y={estimated_pose[1]:.2f},theta={estimated_pose[2]:.2f},error={error:.2f}"
        self.status_pub.publish(status_msg)

        # Publish TF transform
        self.publish_tf_transform()

        # Log information periodically
        if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
            self.get_logger().info(f'Estimated pose: ({estimated_pose[0]:.2f}, {estimated_pose[1]:.2f}, {estimated_pose[2]:.2f}), Error: {error:.2f}m')

    def publish_pose_estimate(self):
        """Publish the estimated pose."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = self.current_pose['x']
        pose_msg.pose.pose.position.y = self.current_pose['y']
        pose_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = quaternion_from_euler(0, 0, self.current_pose['theta'])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # Set covariance (diagonal elements only for simplicity)
        # This would be calculated from particle distribution in real implementation
        pose_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        self.pose_pub.publish(pose_msg)

    def publish_tf_transform(self):
        """Publish TF transform for the robot pose."""
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # Set translation
        t.transform.translation.x = self.current_pose['x']
        t.transform.translation.y = self.current_pose['y']
        t.transform.translation.z = 0.0

        # Set rotation
        quat = quaternion_from_euler(0, 0, self.current_pose['theta'])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def switch_localization_method(self, method: str):
        """Switch between different localization methods."""
        if method in ['particle_filter', 'ekf', 'amcl']:
            self.localization_method = method
            self.get_logger().info(f'Switched to {method} localization method')

            # Reinitialize particles if switching to particle-based method
            if method in ['particle_filter', 'amcl']:
                self.initialize_particles()
        else:
            self.get_logger().warn(f'Unknown localization method: {method}')

    def get_localization_stats(self) -> Dict[str, Any]:
        """Get statistics about localization performance."""
        stats = {
            'method': self.localization_method,
            'num_particles': self.num_particles,
            'particles_variance': np.var(self.particles, axis=0).tolist(),
            'current_pose': self.current_pose.copy(),
            'ground_truth_pose': self.ground_truth_pose.copy(),
            'particle_distribution': {
                'x_mean': np.mean(self.particles[:, 0]),
                'y_mean': np.mean(self.particles[:, 1]),
                'theta_mean': np.mean(self.particles[:, 2]),
                'x_std': np.std(self.particles[:, 0]),
                'y_std': np.std(self.particles[:, 1]),
                'theta_std': np.std(self.particles[:, 2])
            }
        }
        return stats


class LocalizationEvaluator:
    """
    Evaluator for localization performance.
    """

    def __init__(self, node: LocalizationNode):
        self.node = node
        self.errors = []
        self.timestamps = []

    def evaluate_localization_accuracy(self, num_samples: int = 100) -> Dict[str, float]:
        """Evaluate localization accuracy over time."""
        self.get_logger().info(f'Evaluating localization accuracy over {num_samples} samples')

        # This would normally run for a specified time or number of samples
        # For this example, we'll return placeholder statistics
        if len(self.errors) > 0:
            mean_error = np.mean(self.errors)
            std_error = np.std(self.errors)
            max_error = np.max(self.errors)
            min_error = np.min(self.errors)
        else:
            mean_error = 0.0
            std_error = 0.0
            max_error = 0.0
            min_error = 0.0

        stats = {
            'mean_error': mean_error,
            'std_error': std_error,
            'max_error': max_error,
            'min_error': min_error,
            'rmse': math.sqrt(mean_error**2 + std_error**2) if len(self.errors) > 0 else 0.0
        }

        return stats

    def get_logger(self):
        """Helper to access logger."""
        return self.node.get_logger()


class MultiSensorFusionLocalizer(LocalizationNode):
    """
    Advanced localizer that fuses multiple sensors for improved localization.
    """

    def __init__(self):
        super().__init__()

        # Additional sensor subscribers
        self.visual_odom_sub = self.create_subscription(
            Odometry, '/visual_odom', self.visual_odom_callback, 10
        )
        self.gps_sub = self.create_subscription(
            String, '/gps_fix', self.gps_callback, 10  # Simplified GPS as string
        )

        # Sensor fusion weights
        self.fusion_weights = {
            'laser': 0.5,
            'visual': 0.3,
            'imu': 0.2
        }

        self.visual_odom_data = None
        self.gps_data = None

        self.get_logger().info('Multi-sensor Fusion Localizer initialized')

    def visual_odom_callback(self, msg):
        """Process visual odometry data."""
        self.visual_odom_data = msg

    def gps_callback(self, msg):
        """Process GPS data."""
        self.gps_data = msg.data  # Simplified GPS as string

    def fused_localization(self, control):
        """Perform localization using multiple sensors."""
        # Get estimates from different sensors
        laser_estimate = self.particle_filter_localization(control)

        # In a real implementation, you would get estimates from other sensors
        # and fuse them using techniques like Kalman filtering or particle weighting
        fused_estimate = laser_estimate  # Placeholder

        # Adjust weights based on sensor reliability
        # In real implementation, this would consider sensor noise models
        # and environmental conditions

        return fused_estimate


def main(args=None):
    """Main function to run localization examples."""
    rclpy.init(args=args)

    # Create the localization node
    localization_node = LocalizationNode()

    # Create evaluator
    evaluator = LocalizationEvaluator(localization_node)

    try:
        # Switch to different localization methods to demonstrate
        localization_node.switch_localization_method('particle_filter')

        # Run for a period of time
        start_time = time.time()
        duration = 30  # seconds

        while time.time() - start_time < duration:
            rclpy.spin_once(localization_node, timeout_sec=0.1)

            # Periodically switch methods to demonstrate
            if int(time.time() - start_time) % 10 == 5:  # Switch every 10 seconds
                methods = ['particle_filter', 'ekf', 'amcl']
                current_idx = methods.index(localization_node.localization_method)
                next_method = methods[(current_idx + 1) % len(methods)]
                localization_node.switch_localization_method(next_method)

        # Get final statistics
        stats = localization_node.get_localization_stats()
        localization_node.get_logger().info(f'Localization statistics: {stats}')

        # Evaluate performance
        perf_stats = evaluator.evaluate_localization_accuracy()
        localization_node.get_logger().info(f'Performance statistics: {perf_stats}')

    except KeyboardInterrupt:
        localization_node.get_logger().info('Localization node stopped by user')
    finally:
        localization_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()