#!/usr/bin/env python3

"""
Nav2 Navigation Examples for the Physical AI & Humanoid Robotics Course
This module demonstrates various navigation concepts using ROS 2 Navigation (Nav2).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes
from std_msgs.msg import String, Bool

import math
import time
from typing import List, Tuple, Dict, Any


class Nav2Examples(Node):
    """
    Example implementations of Nav2 navigation concepts and patterns.
    """

    def __init__(self):
        super().__init__('nav2_examples')

        # Action clients for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # Publishers and subscribers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Services
        self.load_map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.lifecycle_client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager/manage_nodes')

        # Navigation state
        self.current_goal = None
        self.navigation_active = False
        self.waypoints = []
        self.current_pose = None

        # Example navigation patterns
        self.navigation_patterns = {
            'square_patrol': self.square_patrol_pattern,
            'random_walk': self.random_walk_pattern,
            'boundary_follow': self.boundary_follow_pattern,
            'goal_sequence': self.goal_sequence_pattern
        }

        self.get_logger().info('Nav2 Examples initialized')

    def send_initial_pose(self, x: float, y: float, theta: float = 0.0):
        """Send initial pose estimate to Nav2."""
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()

        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        initial_pose_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        initial_pose_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        initial_pose_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Set covariance to identity matrix (high uncertainty)
        initial_pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        self.initial_pose_pub.publish(initial_pose_msg)
        self.get_logger().info(f'Initial pose sent: ({x}, {y}, {theta})')

    def send_goal_pose(self, x: float, y: float, theta: float = 0.0) -> bool:
        """Send a single navigation goal."""
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            return False

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        self.current_goal = goal_msg
        self.navigation_active = True

        future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'Goal sent: ({x}, {y}, {theta})')

        return True

    def send_waypoints(self, waypoints: List[Tuple[float, float, float]]) -> bool:
        """Send multiple waypoints for navigation."""
        if not self.nav_through_poses_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateThroughPoses action server not available')
            return False

        # Create goal message
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        for i, (x, y, theta) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)

            goal_msg.poses.append(pose)

        # Send waypoints
        self.waypoints = waypoints
        self.navigation_active = True

        future = self.nav_through_poses_client.send_goal_async(
            goal_msg,
            feedback_callback=self.waypoint_feedback_callback
        )

        future.add_done_callback(self.waypoint_response_callback)
        self.get_logger().info(f'Sent {len(waypoints)} waypoints')

        return True

    def goal_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.navigation_active = False

    def waypoint_response_callback(self, future):
        """Handle waypoint navigation response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Waypoints rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Waypoints accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result_callback)

    def waypoint_result_callback(self, future):
        """Handle waypoint navigation result."""
        result = future.result().result
        self.get_logger().info(f'Waypoint navigation result: {result}')
        self.navigation_active = False

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation progress: {feedback.distance_remaining:.2f}m remaining')

    def waypoint_feedback_callback(self, feedback_msg):
        """Handle waypoint navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Waypoint progress: {feedback.current_waypoint + 1}/{len(self.waypoints)}')

    def square_patrol_pattern(self, center_x: float = 0.0, center_y: float = 0.0, size: float = 2.0):
        """Implement a square patrol pattern."""
        self.get_logger().info('Executing square patrol pattern')

        # Define square corners
        corners = [
            (center_x + size/2, center_y + size/2, math.pi/4),  # Top-right
            (center_x - size/2, center_y + size/2, 3*math.pi/4),  # Top-left
            (center_x - size/2, center_y - size/2, -3*math.pi/4),  # Bottom-left
            (center_x + size/2, center_y - size/2, -math.pi/4)   # Bottom-right
        ]

        # Navigate to each corner in sequence
        for i, (x, y, theta) in enumerate(corners):
            self.get_logger().info(f'Navigating to corner {i+1}: ({x}, {y})')

            if not self.send_goal_pose(x, y, theta):
                self.get_logger().error(f'Failed to send goal to corner {i+1}')
                break

            # Wait for navigation to complete
            while self.navigation_active:
                time.sleep(0.1)

            # Wait a bit before moving to next corner
            time.sleep(1.0)

        self.get_logger().info('Square patrol pattern completed')

    def random_walk_pattern(self, steps: int = 10, step_size: float = 1.0):
        """Implement a random walk navigation pattern."""
        self.get_logger().info('Executing random walk pattern')

        current_x, current_y = 0.0, 0.0

        for i in range(steps):
            # Generate random direction
            angle = 2 * math.pi * (i / steps)  # Deterministic for demo
            dx = step_size * math.cos(angle)
            dy = step_size * math.sin(angle)

            target_x = current_x + dx
            target_y = current_y + dy

            self.get_logger().info(f'Step {i+1}: Moving to ({target_x:.2f}, {target_y:.2f})')

            if not self.send_goal_pose(target_x, target_y, angle):
                self.get_logger().error(f'Failed to send goal for step {i+1}')
                break

            # Wait for navigation to complete
            while self.navigation_active:
                time.sleep(0.1)

            current_x, current_y = target_x, target_y
            time.sleep(0.5)  # Pause between steps

        self.get_logger().info('Random walk pattern completed')

    def boundary_follow_pattern(self, center_x: float = 0.0, center_y: float = 0.0, radius: float = 3.0, steps: int = 12):
        """Implement a boundary following pattern."""
        self.get_logger().info('Executing boundary follow pattern')

        for i in range(steps):
            angle = 2 * math.pi * i / steps
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)

            self.get_logger().info(f'Following boundary - point {i+1}: ({x:.2f}, {y:.2f})')

            if not self.send_goal_pose(x, y, angle + math.pi/2):  # Face tangent direction
                self.get_logger().error(f'Failed to send goal for boundary point {i+1}')
                break

            # Wait for navigation to complete
            while self.navigation_active:
                time.sleep(0.1)

            time.sleep(0.5)

        self.get_logger().info('Boundary follow pattern completed')

    def goal_sequence_pattern(self, goals: List[Tuple[float, float, float]]):
        """Execute a predefined sequence of goals."""
        self.get_logger().info('Executing goal sequence pattern')

        for i, (x, y, theta) in enumerate(goals):
            self.get_logger().info(f'Moving to goal {i+1}: ({x}, {y}, {theta})')

            if not self.send_goal_pose(x, y, theta):
                self.get_logger().error(f'Failed to send goal {i+1}')
                break

            # Wait for navigation to complete
            while self.navigation_active:
                time.sleep(0.1)

            time.sleep(1.0)  # Pause at each goal

        self.get_logger().info('Goal sequence pattern completed')

    def create_simple_path_plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float, float]]:
        """Create a simple path plan between start and goal."""
        start_x, start_y = start
        goal_x, goal_y = goal

        # Calculate distance and angle
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)

        # Create intermediate waypoints
        waypoints = []
        num_waypoints = max(2, int(distance / 0.5))  # 0.5m spacing

        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            x = start_x + t * dx
            y = start_y + t * dy
            waypoints.append((x, y, angle))

        return waypoints

    def execute_path_plan(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """Execute a path plan from start to goal."""
        self.get_logger().info(f'Planning path from {start} to {goal}')

        waypoints = self.create_simple_path_plan(start, goal)

        self.get_logger().info(f'Path plan created with {len(waypoints)} waypoints')

        if not self.send_waypoints(waypoints):
            self.get_logger().error('Failed to send path plan')
            return

        # Wait for navigation to complete
        while self.navigation_active:
            time.sleep(0.1)

        self.get_logger().info('Path plan execution completed')

    def test_navigation_reliability(self, num_tests: int = 5):
        """Test navigation reliability with repeated goals."""
        self.get_logger().info(f'Testing navigation reliability with {num_tests} tests')

        test_goals = [
            (2.0, 2.0, 0.0),
            (-2.0, 2.0, math.pi/2),
            (-2.0, -2.0, math.pi),
            (2.0, -2.0, -math.pi/2),
            (0.0, 0.0, 0.0)  # Return to start
        ]

        success_count = 0
        total_attempts = min(num_tests, len(test_goals))

        for i in range(total_attempts):
            goal = test_goals[i]
            self.get_logger().info(f'Navigation test {i+1}: attempting to reach {goal}')

            success = self.send_goal_pose(*goal)
            if success:
                # Wait for completion
                start_time = time.time()
                timeout = 30  # 30 second timeout per test

                while self.navigation_active and (time.time() - start_time) < timeout:
                    time.sleep(0.1)

                if not self.navigation_active:
                    success_count += 1
                    self.get_logger().info(f'Test {i+1} succeeded')
                else:
                    self.get_logger().warning(f'Test {i+1} timed out')
            else:
                self.get_logger().error(f'Test {i+1} failed to send goal')

            time.sleep(2.0)  # Wait between tests

        success_rate = success_count / total_attempts if total_attempts > 0 else 0
        self.get_logger().info(f'Navigation reliability test: {success_count}/{total_attempts} successes ({success_rate*100:.1f}%)')

    def demonstrate_adaptive_navigation(self):
        """Demonstrate adaptive navigation with dynamic replanning."""
        self.get_logger().info('Demonstrating adaptive navigation')

        # This would normally involve:
        # 1. Monitoring sensor data for dynamic obstacles
        # 2. Detecting when path becomes invalid
        # 3. Replanning to goal with new information
        # 4. Executing updated plan

        # For this example, we'll simulate the process
        self.get_logger().info('Simulating adaptive navigation scenario...')

        # Initial plan
        self.get_logger().info('Initial path planned to goal')

        # Simulate obstacle detection
        time.sleep(2.0)
        self.get_logger().info('Dynamic obstacle detected, replanning...')

        # Simulate replanning
        time.sleep(1.0)
        self.get_logger().info('New path computed, continuing navigation...')

        # Simulate reaching goal
        time.sleep(2.0)
        self.get_logger().info('Goal reached with adaptive navigation')

    def run_navigation_example(self, example_type: str = 'square_patrol'):
        """Run a specific navigation example."""
        if example_type in self.navigation_patterns:
            self.get_logger().info(f'Running {example_type} navigation example')
            self.navigation_patterns[example_type]()
        else:
            self.get_logger().error(f'Unknown navigation example: {example_type}')
            self.get_logger().info(f'Available examples: {list(self.navigation_patterns.keys())}')


class Nav2Navigator(Node):
    """
    A simple navigator that demonstrates Nav2 integration.
    """

    def __init__(self):
        super().__init__('nav2_navigator')

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer for navigation tasks
        self.nav_timer = self.create_timer(1.0, self.navigation_task)

        # Navigation parameters
        self.navigation_targets = [
            (1.0, 1.0, 0.0),
            (2.0, 0.0, math.pi/2),
            (1.0, -1.0, math.pi),
            (0.0, 0.0, -math.pi/2)
        ]
        self.current_target_idx = 0

        self.get_logger().info('Nav2 Navigator initialized')

    def navigation_task(self):
        """Execute navigation tasks in sequence."""
        if self.current_target_idx >= len(self.navigation_targets):
            self.get_logger().info('All navigation targets completed')
            return

        target = self.navigation_targets[self.current_target_idx]
        x, y, theta = target

        self.get_logger().info(f'Navigating to target {self.current_target_idx + 1}: ({x}, {y}, {theta})')

        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('NavigateToPose action server not available, retrying...')
            return

        # Create and send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        """Handle navigation result."""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

        # Move to next target
        self.current_target_idx += 1


def main(args=None):
    """Main function to run Nav2 examples."""
    rclpy.init(args=args)

    # Create the examples node
    nav2_examples = Nav2Examples()

    try:
        # Example: Send initial pose
        nav2_examples.send_initial_pose(0.0, 0.0, 0.0)

        # Run a specific navigation example
        nav2_examples.run_navigation_example('square_patrol')

        # Example: Test navigation reliability
        # nav2_examples.test_navigation_reliability(3)

        # Example: Demonstrate adaptive navigation
        # nav2_examples.demonstrate_adaptive_navigation()

        # You can also create a navigator that runs continuously
        # navigator = Nav2Navigator()
        # rclpy.spin(nav2_examples)

    except KeyboardInterrupt:
        nav2_examples.get_logger().info('Nav2 examples stopped by user')
    finally:
        nav2_examples.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()