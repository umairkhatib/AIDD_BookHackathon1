#!/usr/bin/env python3

"""
Basic Navigation Scene Configuration for VLA Demo
This module sets up a basic navigation scene in Isaac Sim for VLA demonstrations.
"""

import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.isaac.core.materials import OmniPBRMaterial
from omni.isaac.core.utils.materials import create_material
from omni.isaac.sensor import Camera, RotatingLidarSensor
from pxr import Gf, UsdGeom, Sdf
import numpy as np
import math


class BasicNavigationScene:
    """
    Basic Navigation Scene for VLA Demo.
    Creates a simple indoor environment with navigation targets.
    """

    def __init__(self):
        # Initialize the world
        self.world = World(stage_units_in_meters=1.0)

        # Scene parameters
        self.room_size = 10.0  # meters
        self.robot_start_pos = np.array([0.0, 0.0, 0.2])

        # Navigation targets
        self.navigation_targets = {
            "kitchen": np.array([4.0, 3.0, 0.0]),
            "living_room": np.array([0.0, 0.0, 0.0]),
            "bedroom": np.array([-3.0, 2.0, 0.0]),
            "office": np.array([2.0, -3.0, 0.0]),
            "entrance": np.array([0.0, 4.0, 0.0])
        }

        print("Basic Navigation Scene initialized")

    def setup_environment(self):
        """Set up the basic navigation environment."""
        print("Setting up navigation environment...")

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Create room boundaries
        self._create_room_boundaries()

        # Add navigation targets
        self._add_navigation_targets()

        # Add obstacles for navigation challenge
        self._add_navigation_obstacles()

        # Add a simple robot (in a real scenario, you would add a proper robot model)
        self._add_simple_robot()

        print("Navigation environment setup complete")

    def _create_room_boundaries(self):
        """Create walls for the navigation room."""
        wall_thickness = 0.2
        wall_height = 2.0

        # Calculate wall positions
        room_half = self.room_size / 2.0

        # Create walls with different colors for visual distinction
        walls = [
            {"name": "NorthWall", "pos": [0, room_half, wall_height/2], "rotation": [0.707, 0, 0, 0.707], "color": [0.5, 0.5, 0.5]},
            {"name": "SouthWall", "pos": [0, -room_half, wall_height/2], "rotation": [0.707, 0, 0, 0.707], "color": [0.5, 0.5, 0.5]},
            {"name": "EastWall", "pos": [room_half, 0, wall_height/2], "rotation": [0.707, 0, 0.707, 0], "color": [0.5, 0.5, 0.5]},
            {"name": "WestWall", "pos": [-room_half, 0, wall_height/2], "rotation": [0.707, 0, 0.707, 0], "color": [0.5, 0.5, 0.5]}
        ]

        for wall in walls:
            create_prim(
                prim_path=f"/World/{wall['name']}",
                prim_type="Cylinder",  # Using cylinder with appropriate rotation to create walls
                position=np.array(wall['pos']),
                orientation=np.array(wall['rotation']),
                scale=np.array([wall_thickness, self.room_size, wall_height]),
                attributes={"radius": wall_thickness/2, "height": wall_height}
            )

    def _add_navigation_targets(self):
        """Add navigation targets to the scene."""
        print("Adding navigation targets...")

        for name, pos in self.navigation_targets.items():
            # Create a visual marker for the target
            target_marker = self.world.scene.add(
                DynamicSphere(
                    prim_path=f"/World/Targets/{name}_target",
                    name=f"{name}_target",
                    position=pos + np.array([0, 0, 0.2]),  # Slightly above ground
                    radius=0.2,
                    color=np.array([0.8, 0.8, 0.2]) if name == "kitchen" else
                          np.array([0.2, 0.8, 0.2]) if name == "living_room" else
                          np.array([0.2, 0.2, 0.8]) if name == "bedroom" else
                          np.array([0.8, 0.2, 0.8]) if name == "office" else
                          np.array([0.8, 0.2, 0.2])  # entrance
                )
            )

    def _add_navigation_obstacles(self):
        """Add obstacles to make navigation challenging."""
        print("Adding navigation obstacles...")

        obstacles = [
            {"name": "Obstacle1", "pos": [1.5, 1.5, 0.3], "size": 0.4, "color": [0.8, 0.2, 0.2]},
            {"name": "Obstacle2", "pos": [-1.5, -1.5, 0.3], "size": 0.5, "color": [0.2, 0.8, 0.2]},
            {"name": "Obstacle3", "pos": [2.0, -1.0, 0.3], "size": 0.3, "color": [0.2, 0.2, 0.8]},
            {"name": "Obstacle4", "pos": [-2.0, 1.0, 0.3], "size": 0.4, "color": [0.8, 0.8, 0.2]},
        ]

        for i, obs in enumerate(obstacles):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Obstacles/Obstacle_{i+1}",
                    name=f"obstacle_{i+1}",
                    position=np.array(obs["pos"]),
                    size=obs["size"],
                    color=np.array(obs["color"])
                )
            )

    def _add_simple_robot(self):
        """Add a simple robot to the scene."""
        print("Adding simple robot to scene...")

        # Create a simple robot represented as a cuboid with sensors
        robot = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Robot",
                name="simple_robot",
                position=self.robot_start_pos,
                size=0.3,
                color=np.array([0.1, 0.8, 0.1])  # Green color
            )
        )

        # Add a camera to the robot for visual perception
        camera = Camera(
            prim_path="/World/Robot/Camera",
            position=np.array([0.2, 0, 0.1]),
            frequency=30,
            resolution=(640, 480)
        )

        # Add a simple LIDAR to the robot
        lidar = RotatingLidarSensor(
            prim_path="/World/Robot/Lidar",
            translation=np.array([0.2, 0, 0.2]),
            name="RobotLidar",
            fov=360,
            horizontal_resolution=1,
            vertical_resolution=16,
            range=10,
            rotation_frequency=20,
            points_per_second=50000
        )

    def run_scene(self, num_steps=1000):
        """Run the navigation scene simulation."""
        print(f"Running navigation scene for {num_steps} steps...")

        self.world.reset()

        for i in range(num_steps):
            self.world.step(render=True)

            # Print progress periodically
            if i % 200 == 0:
                print(f"Scene simulation step: {i}")

                # In a real navigation scenario, you would:
                # - Collect sensor data
                # - Process perception information
                # - Plan navigation paths
                # - Execute navigation commands

        print("Navigation scene simulation completed!")

    def get_scene_info(self):
        """Get information about the current scene."""
        info = {
            "room_size": self.room_size,
            "robot_start": self.robot_start_pos.tolist(),
            "navigation_targets": {name: pos.tolist() for name, pos in self.navigation_targets.items()},
            "num_obstacles": 4,
            "stage_units": 1.0  # Default to 1.0 if function is not available
        }
        return info


def main():
    """Main function to create and run the basic navigation scene."""
    print("Creating Basic Navigation Scene...")

    scene = BasicNavigationScene()

    try:
        # Set up the environment
        scene.setup_environment()

        # Print scene information
        info = scene.get_scene_info()
        print(f"\nScene Information:")
        print(f"  Room size: {info['room_size']}m x {info['room_size']}m")
        print(f"  Robot start: {info['robot_start']}")
        print(f"  Navigation targets: {list(info['navigation_targets'].keys())}")
        print(f"  Number of obstacles: {info['num_obstacles']}")

        # Run the scene
        scene.run_scene()

    except Exception as e:
        print(f"Error during scene execution: {e}")
        carb.log_error(str(e))
    finally:
        # Cleanup
        scene.world.clear()
        print("\nScene cleared and resources released")


if __name__ == "__main__":
    main()