#!/usr/bin/env python3

"""
Perception Pipeline for the Physical AI & Humanoid Robotics Course
This module implements a complete perception pipeline for robotics applications.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from typing import List, Tuple, Dict, Any
import tensorflow as tf  # Placeholder - would use Isaac ROS in real implementation
from scipy.spatial.transform import Rotation as R
import time


class PerceptionPipeline(Node):
    """
    A comprehensive perception pipeline that processes various sensor inputs
    and produces semantic understanding of the environment.
    """

    def __init__(self):
        super().__init__('perception_pipeline')

        # Create CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Publishers
        self.object_detection_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.semantic_map_pub = self.create_publisher(String, '/perception/semantic_map', 10)
        self.obstacle_map_pub = self.create_publisher(PointCloud2, '/perception/obstacle_map', 10)
        self.status_pub = self.create_publisher(String, '/perception/status', 10)

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(String, '/imu/data', self.imu_callback, 10)

        # Perception state
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_lidar = None
        self.latest_imu = None

        # Perception results
        self.detected_objects = []
        self.semantic_map = None
        self.obstacle_map = None

        # Processing parameters
        self.detection_confidence_threshold = 0.7
        self.min_object_size = 10  # pixels
        self.max_detection_range = 10.0  # meters

        # Timer for perception processing
        self.perception_timer = self.create_timer(0.1, self.process_perception)  # 10Hz

        self.get_logger().info('Perception Pipeline initialized')

    def rgb_callback(self, msg):
        """Process RGB camera data."""
        try:
            # Convert ROS Image to OpenCV image
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform basic preprocessing
            self.preprocess_rgb_image(self.latest_rgb)

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg):
        """Process depth camera data."""
        try:
            # Convert ROS Image to OpenCV image (depth)
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, "32FC1")

            # Validate depth data
            self.validate_depth_data(self.latest_depth)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def lidar_callback(self, msg):
        """Process LIDAR scan data."""
        self.latest_lidar = msg

        # Process LIDAR data for obstacle detection
        self.process_lidar_for_obstacles(msg)

    def imu_callback(self, msg):
        """Process IMU data."""
        self.latest_imu = msg

    def preprocess_rgb_image(self, image):
        """Preprocess RGB image for object detection."""
        # Resize image if needed for processing
        if image.shape[0] > 640 or image.shape[1] > 640:
            # Maintain aspect ratio
            h, w = image.shape[:2]
            scale = min(640.0 / h, 640.0 / w)
            new_w, new_h = int(w * scale), int(h * scale)
            image = cv2.resize(image, (new_w, new_h))

        # Apply basic preprocessing
        # In a real implementation, this would include normalization, etc.
        return image

    def validate_depth_data(self, depth_image):
        """Validate and clean depth data."""
        if depth_image is None:
            return

        # Replace invalid values with reasonable defaults
        depth_image = np.nan_to_num(depth_image, nan=np.inf, posinf=np.inf, neginf=-np.inf)

        # Apply median filter to reduce noise
        filtered_depth = cv2.medianBlur(depth_image.astype(np.float32), 5)

        # Update with filtered data
        self.latest_depth = filtered_depth

    def process_lidar_for_obstacles(self, lidar_msg):
        """Process LIDAR data to detect obstacles."""
        if not lidar_msg.ranges:
            return

        # Convert LIDAR ranges to obstacle points
        obstacle_points = []
        angle_increment = lidar_msg.angle_increment
        angle = lidar_msg.angle_min

        for i, range_val in enumerate(lidar_msg.ranges):
            if 0 < range_val < self.max_detection_range:  # Valid range
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)

                # Only include points within reasonable bounds
                if abs(x) < 10 and abs(y) < 10:
                    obstacle_points.append([x, y, 0.0])  # z=0 for ground level

            angle += angle_increment

        self.obstacle_map = obstacle_points

    def detect_objects(self, image):
        """Detect objects in the image using a placeholder detector."""
        # Placeholder for object detection
        # In a real implementation, this would use Isaac ROS DetectNet or similar
        detections = []

        # For demonstration, we'll create some synthetic detections
        height, width = image.shape[:2]

        # Create synthetic detections based on visual features
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Use simple blob detection as a placeholder
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_object_size:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate confidence based on size (larger objects more confident)
                confidence = min(0.9, area / (width * height))

                if confidence > self.detection_confidence_threshold:
                    detection = {
                        'bbox': (x, y, x + w, y + h),
                        'label': 'object',  # In real implementation, would have specific labels
                        'confidence': confidence,
                        'center': (x + w/2, y + h/2)
                    }
                    detections.append(detection)

        return detections

    def segment_semantics(self, image, depth_image=None):
        """Perform semantic segmentation on the image."""
        # Placeholder for semantic segmentation
        # In a real implementation, this would use Isaac ROS segmentation
        height, width = image.shape[:2]

        # Create a simple semantic map based on color regions
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for different semantic classes
        color_ranges = {
            'red': ([0, 50, 50], [10, 255, 255]),
            'blue': ([100, 50, 50], [130, 255, 255]),
            'green': ([40, 50, 50], [80, 255, 255]),
            'yellow': ([20, 50, 50], [40, 255, 255])
        }

        semantic_map = np.zeros((height, width), dtype=np.uint8)

        for class_id, (lower, upper) in enumerate(color_ranges.values(), start=1):
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            semantic_map[mask > 0] = class_id

        return semantic_map

    def fuse_sensors(self):
        """Fuse information from multiple sensors."""
        fused_data = {
            'objects': self.detected_objects,
            'semantic_map': self.semantic_map,
            'obstacles': self.obstacle_map,
            'timestamp': time.time()
        }

        # Perform sensor fusion
        # In a real implementation, this would use advanced fusion techniques
        if self.latest_rgb is not None and self.latest_depth is not None:
            # Associate detected objects with depth information
            for obj in fused_data['objects']:
                bbox = obj['bbox']
                center_x, center_y = int(obj['center'][0]), int(obj['center'][1])

                # Get depth at object center
                if 0 <= center_y < self.latest_depth.shape[0] and 0 <= center_x < self.latest_depth.shape[1]:
                    depth_value = self.latest_depth[center_y, center_x]
                    obj['distance'] = depth_value

        return fused_data

    def create_detection_message(self, detections):
        """Create a Detection2DArray message from detection results."""
        detection_array_msg = Detection2DArray()
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()
        detection_array_msg.header.frame_id = 'camera_frame'

        for det in detections:
            if det['confidence'] >= self.detection_confidence_threshold:
                detection_msg = Detection2D()

                # Set bounding box
                x1, y1, x2, y2 = det['bbox']
                detection_msg.bbox.center.x = (x1 + x2) / 2.0
                detection_msg.bbox.center.y = (y1 + y2) / 2.0
                detection_msg.bbox.size_x = abs(x2 - x1)
                detection_msg.bbox.size_y = abs(y2 - y1)

                # Set hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = det['label']
                hypothesis.score = det['confidence']

                detection_msg.results.append(hypothesis)
                detection_array_msg.detections.append(detection_msg)

        return detection_array_msg

    def process_perception(self):
        """Main perception processing loop."""
        if self.latest_rgb is None:
            return

        # Perform object detection
        self.detected_objects = self.detect_objects(self.latest_rgb)

        # Perform semantic segmentation
        if self.latest_depth is not None:
            self.semantic_map = self.segment_semantics(self.latest_rgb, self.latest_depth)
        else:
            self.semantic_map = self.segment_semantics(self.latest_rgb)

        # Fuse sensor data
        fused_data = self.fuse_sensors()

        # Publish detection results
        if self.detected_objects:
            detection_msg = self.create_detection_message(self.detected_objects)
            self.object_detection_pub.publish(detection_msg)

        # Publish semantic map information
        semantic_info_msg = String()
        semantic_info_msg.data = f"Detected {len(self.detected_objects)} objects, semantic map updated"
        self.semantic_map_pub.publish(semantic_info_msg)

        # Publish status
        status_msg = String()
        status_msg.data = "PERCEPTION_ACTIVE"
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Processed perception: {len(self.detected_objects)} objects detected')


class PerceptionManager:
    """
    Higher-level manager for perception pipeline coordination.
    """

    def __init__(self, node: PerceptionPipeline):
        self.node = node
        self.perception_pipeline = node
        self.active_modules = set(['object_detection', 'semantic_segmentation', 'obstacle_detection'])

    def activate_module(self, module_name: str):
        """Activate a specific perception module."""
        self.active_modules.add(module_name)

    def deactivate_module(self, module_name: str):
        """Deactivate a specific perception module."""
        self.active_modules.discard(module_name)

    def get_perception_results(self) -> Dict[str, Any]:
        """Get current perception results."""
        return {
            'detected_objects': self.node.detected_objects,
            'semantic_map': self.node.semantic_map,
            'obstacles': self.node.obstacle_map,
            'status': 'active'
        }

    def configure_pipeline(self, params: Dict[str, Any]):
        """Configure perception pipeline parameters."""
        if 'confidence_threshold' in params:
            self.node.detection_confidence_threshold = params['confidence_threshold']
        if 'min_object_size' in params:
            self.node.min_object_size = params['min_object_size']
        if 'max_detection_range' in params:
            self.node.max_detection_range = params['max_detection_range']


def main(args=None):
    """Main function to run the perception pipeline."""
    rclpy.init(args=args)

    perception_pipeline = PerceptionPipeline()

    # Create perception manager
    perception_manager = PerceptionManager(perception_pipeline)

    # Example configuration
    config = {
        'confidence_threshold': 0.7,
        'min_object_size': 15,
        'max_detection_range': 8.0
    }
    perception_manager.configure_pipeline(config)

    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        perception_pipeline.get_logger().info('Perception pipeline stopped by user')
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()