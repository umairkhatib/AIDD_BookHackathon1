#!/usr/bin/env python3

"""
Sensor Integration Configuration for Isaac Sim
This example demonstrates how to configure various sensors in Isaac Sim for perception tasks.
"""

import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera, RotatingLidarSensor
from pxr import Gf, UsdGeom, Sdf
import numpy as np
import math


class SensorIntegrationScene:
    def __init__(self):
        # Initialize the world
        self.world = World(stage_units_in_meters=1.0)

        # Scene parameters
        self.room_size = 8.0  # meters
        self.robot_pos = np.array([0, 0, 0.3])

        print("Sensor Integration Scene initialized")

    def setup_environment(self):
        """Set up the sensor testing environment."""
        print("Setting up sensor integration environment...")

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Create room boundaries
        self._create_room_boundaries()

        # Add test objects for sensor calibration
        self._add_test_objects()

        # Add robot with sensors
        self._add_robot_with_sensors()

        print("Sensor integration environment setup complete")

    def _create_room_boundaries(self):
        """Create walls with different materials for sensor testing."""
        wall_thickness = 0.1
        wall_height = 2.0

        # Calculate wall positions
        room_half = self.room_size / 2.0

        # Create walls with different materials for sensor testing
        walls = [
            {"name": "NorthWall", "pos": [0, room_half, wall_height/2], "rotation": [0.707, 0, 0, 0.707]},
            {"name": "SouthWall", "pos": [0, -room_half, wall_height/2], "rotation": [0.707, 0, 0, 0.707]},
            {"name": "EastWall", "pos": [room_half, 0, wall_height/2], "rotation": [0.707, 0, 0.707, 0]},
            {"name": "WestWall", "pos": [-room_half, 0, wall_height/2], "rotation": [0.707, 0, 0.707, 0]}
        ]

        for wall in walls:
            create_prim(
                prim_path=f"/World/{wall['name']}",
                prim_type="Cylinder",
                position=np.array(wall['pos']),
                orientation=np.array(wall['rotation']),
                scale=np.array([wall_thickness, self.room_size, wall_height]),
                attributes={"radius": wall_thickness/2, "height": wall_height}
            )

    def _add_test_objects(self):
        """Add objects with different properties for sensor testing."""
        print("Adding test objects for sensor calibration...")

        # Add objects with different materials and properties
        objects = [
            {"name": "RedBox", "pos": [2, 1, 0.3], "size": 0.4, "color": [0.8, 0.2, 0.2]},
            {"name": "GreenSphere", "pos": [-2, 1, 0.4], "size": 0.3, "color": [0.2, 0.8, 0.2]},
            {"name": "BlueCylinder", "pos": [1, -2, 0.5], "size": [0.2, 0.2, 0.8], "color": [0.2, 0.2, 0.8]},
            {"name": "YellowPyramid", "pos": [-1, -2, 0.4], "size": 0.5, "color": [0.8, 0.8, 0.2]},
            {"name": "Mirror", "pos": [0, 3, 1.0], "size": [1.0, 0.02, 1.0], "color": [0.9, 0.9, 0.9]}
        ]

        for obj in objects:
            if obj["name"] == "GreenSphere":
                # Create sphere
                self.world.scene.add(
                    DynamicSphere(
                        prim_path=f"/World/{obj['name']}",
                        name=obj['name'],
                        position=np.array(obj["pos"]),
                        radius=obj["size"],
                        color=np.array(obj["color"])
                    )
                )
            elif obj["name"] == "Mirror":
                # Create reflective surface
                create_prim(
                    prim_path=f"/World/{obj['name']}",
                    prim_type="Cuboid",
                    position=np.array(obj["pos"]),
                    scale=np.array(obj["size"])
                )
            else:
                # Create cuboid
                self.world.scene.add(
                    DynamicCuboid(
                        prim_path=f"/World/{obj['name']}",
                        name=obj['name'],
                        position=np.array(obj["pos"]),
                        size=obj["size"],
                        color=np.array(obj["color"])
                    )
                )

    def _add_robot_with_sensors(self):
        """Add a robot equipped with various sensors."""
        print("Adding robot with sensors...")

        # Add robot base
        create_prim(
            prim_path="/World/RobotBase",
            prim_type="Cuboid",
            position=self.robot_pos,
            scale=np.array([0.5, 0.3, 0.4]),
            attributes={"size": 1.0}
        )

        # Add RGB camera
        self._add_rgb_camera()

        # Add depth camera
        self._add_depth_camera()

        # Add LIDAR sensor
        self._add_lidar_sensor()

        # Add IMU (represented visually)
        self._add_imu_indicator()

        print("Robot with sensors added successfully")

    def _add_rgb_camera(self):
        """Add an RGB camera to the robot."""
        print("Adding RGB camera...")

        # Create camera prim
        camera_path = "/World/RobotBase/RGBCamera"
        cam_pos = self.robot_pos + np.array([0.3, 0, 0.1])  # Position in front and slightly above

        create_prim(
            prim_path=camera_path,
            prim_type="Camera",
            position=cam_pos,
            orientation=np.array([0, 0, 0, 1])  # Looking forward
        )

        # Get the camera prim and configure it
        stage = omni.usd.get_context().get_stage()
        camera_prim = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))

        # Set camera properties
        camera_prim.GetFocalLengthAttr().Set(24.0)  # mm
        camera_prim.GetHorizontalApertureAttr().Set(20.955)  # mm
        camera_prim.GetVerticalApertureAttr().Set(15.2908)  # mm

        print("RGB camera configured")

    def _add_depth_camera(self):
        """Add a depth camera to the robot."""
        print("Adding depth camera...")

        # Create depth camera prim
        depth_camera_path = "/World/RobotBase/DepthCamera"
        depth_pos = self.robot_pos + np.array([0.3, 0, 0.15])  # Slightly above RGB camera

        create_prim(
            prim_path=depth_camera_path,
            prim_type="Camera",
            position=depth_pos,
            orientation=np.array([0, 0, 0, 1])  # Looking forward
        )

        # Configure depth camera properties
        stage = omni.usd.get_context().get_stage()
        depth_cam_prim = UsdGeom.Camera(stage.GetPrimAtPath(depth_camera_path))
        depth_cam_prim.GetFocalLengthAttr().Set(24.0)
        depth_cam_prim.GetHorizontalApertureAttr().Set(20.955)
        depth_cam_prim.GetVerticalApertureAttr().Set(15.2908)

        print("Depth camera configured")

    def _add_lidar_sensor(self):
        """Add a rotating LIDAR sensor to the robot."""
        print("Adding LIDAR sensor...")

        # For this example, we'll create a visual indicator
        # In a real Isaac Sim setup, you would use the RotatingLidarSensor class
        lidar_path = "/World/RobotBase/Lidar"
        lidar_pos = self.robot_pos + np.array([0.3, 0, 0.3])  # On top of robot

        create_prim(
            prim_path=lidar_path,
            prim_type="Cylinder",
            position=lidar_pos,
            scale=np.array([0.05, 0.05, 0.1])  # Small cylinder to represent LIDAR
        )

        print("LIDAR sensor placeholder added")

    def _add_imu_indicator(self):
        """Add an IMU indicator to the robot."""
        print("Adding IMU indicator...")

        # Create a small cube to represent IMU
        imu_path = "/World/RobotBase/IMU"
        imu_pos = self.robot_pos + np.array([0.1, 0, 0.2])  # Center of robot

        create_prim(
            prim_path=imu_path,
            prim_type="Cube",
            position=imu_pos,
            scale=np.array([0.05, 0.05, 0.05])
        )

        print("IMU indicator added")

    def run_sensor_simulation(self, num_steps=500):
        """Run the sensor simulation."""
        print(f"Running sensor simulation for {num_steps} steps...")

        self.world.reset()

        for i in range(num_steps):
            self.world.step(render=True)

            # Print progress periodically
            if i % 100 == 0:
                print(f"Sensor simulation step: {i}")

                # In a real implementation, you would:
                # - Collect data from each sensor
                # - Process sensor data
                # - Calibrate sensors
                # - Verify sensor fusion

        print("Sensor simulation completed!")

    def get_sensor_config(self):
        """Get information about the sensor configuration."""
        config = {
            "cameras": ["RGB", "Depth"],
            "lidar": {"type": "rotating", "fov": 360},
            "imu": True,
            "robot_position": self.robot_pos.tolist()
        }
        return config


def main():
    """Main function to create and run the sensor integration scene."""
    print("Creating Sensor Integration Scene...")

    scene = SensorIntegrationScene()

    try:
        # Set up the environment
        scene.setup_environment()

        # Print sensor configuration
        config = scene.get_sensor_config()
        print(f"\nSensor Configuration:")
        print(f"  Cameras: {config['cameras']}")
        print(f"  LIDAR: {config['lidar']['type']} with {config['lidar']['fov']}° FOV")
        print(f"  IMU: {config['imu']}")
        print(f"  Robot position: {config['robot_position']}")

        # Run the sensor simulation
        scene.run_sensor_simulation()

    except Exception as e:
        print(f"Error during sensor simulation: {e}")
        carb.log_error(str(e))
    finally:
        # Cleanup
        scene.world.clear()
        print("\nScene cleared and resources released")


if __name__ == "__main__":
    main()