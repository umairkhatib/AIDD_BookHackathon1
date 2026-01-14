#!/usr/bin/env python3

"""
Basic Navigation Scene Configuration for Isaac Sim
This example demonstrates how to set up a basic navigation scene in Isaac Sim.
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
import numpy as np
import math


class BasicNavigationScene:
    def __init__(self):
        # Initialize the world
        self.world = World(stage_units_in_meters=1.0)

        # Get assets root path
        self.assets_root_path = get_assets_root_path()

        # Scene parameters
        self.room_size = 10.0  # meters
        self.robot_start_pos = np.array([-4.0, -4.0, 0.2])
        self.goal_pos = np.array([4.0, 4.0, 0.2])

        print("Basic Navigation Scene initialized")

    def setup_environment(self):
        """Set up the basic navigation environment."""
        print("Setting up navigation environment...")

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Create room boundaries
        self._create_room_boundaries()

        # Add navigation obstacles
        self._add_navigation_obstacles()

        # Add navigation goal indicator
        self._add_navigation_goal()

        # Add a simple robot
        self._add_robot()

        print("Navigation environment setup complete")

    def _create_room_boundaries(self):
        """Create walls for the navigation room."""
        wall_thickness = 0.2
        wall_height = 2.0

        # Calculate wall positions
        room_half = self.room_size / 2.0

        # North wall
        create_prim(
            prim_path="/World/NorthWall",
            prim_type="Cylinder",
            position=np.array([0, room_half, wall_height/2]),
            orientation=np.array([0.707, 0, 0, 0.707]),  # Rotate 90 deg around X
            scale=np.array([wall_thickness, self.room_size, wall_height]),
            attributes={"radius": wall_thickness/2, "height": wall_height}
        )

        # South wall
        create_prim(
            prim_path="/World/SouthWall",
            prim_type="Cylinder",
            position=np.array([0, -room_half, wall_height/2]),
            orientation=np.array([0.707, 0, 0, 0.707]),
            scale=np.array([wall_thickness, self.room_size, wall_height]),
            attributes={"radius": wall_thickness/2, "height": wall_height}
        )

        # East wall
        create_prim(
            prim_path="/World/EastWall",
            prim_type="Cylinder",
            position=np.array([room_half, 0, wall_height/2]),
            orientation=np.array([0.707, 0, 0.707, 0]),  # Rotate 90 deg around Z
            scale=np.array([wall_thickness, self.room_size, wall_height]),
            attributes={"radius": wall_thickness/2, "height": wall_height}
        )

        # West wall
        create_prim(
            prim_path="/World/WestWall",
            prim_type="Cylinder",
            position=np.array([-room_half, 0, wall_height/2]),
            orientation=np.array([0.707, 0, 0.707, 0]),
            scale=np.array([wall_thickness, self.room_size, wall_height]),
            attributes={"radius": wall_thickness/2, "height": wall_height}
        )

    def _add_navigation_obstacles(self):
        """Add obstacles for navigation testing."""
        print("Adding navigation obstacles...")

        # Add some random obstacles
        obstacles = [
            {"pos": [0, 2, 0.3], "size": 0.5, "color": [0.8, 0.2, 0.2]},  # Red obstacle
            {"pos": [-2, 0, 0.3], "size": 0.4, "color": [0.2, 0.8, 0.2]},  # Green obstacle
            {"pos": [2, -2, 0.4], "size": 0.6, "color": [0.2, 0.2, 0.8]},  # Blue obstacle
            {"pos": [1, 3, 0.2], "size": 0.3, "color": [0.8, 0.8, 0.2]}   # Yellow obstacle
        ]

        for i, obs in enumerate(obstacles):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Obstacle_{i}",
                    name=f"obstacle_{i}",
                    position=np.array(obs["pos"]),
                    size=obs["size"],
                    color=np.array(obs["color"])
                )
            )

    def _add_navigation_goal(self):
        """Add a goal indicator for navigation."""
        print("Adding navigation goal...")

        # Add a distinctive goal marker
        self.world.scene.add(
            DynamicSphere(
                prim_path="/World/NavigationGoal",
                name="navigation_goal",
                position=self.goal_pos,
                radius=0.3,
                color=np.array([0.9, 0.7, 0.1])  # Yellow/amber color
            )
        )

    def _add_robot(self):
        """Add a simple robot to the scene."""
        print("Adding robot to scene...")

        # Add a simple robot represented as a cuboid
        # In a real scenario, you would add a proper robot model
        robot = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Robot",
                name="simple_robot",
                position=self.robot_start_pos,
                size=0.4,
                color=np.array([0.1, 0.8, 0.1])  # Green color
            )
        )

        print(f"Robot added at position: {self.robot_start_pos}")

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
            "goal_position": self.goal_pos.tolist(),
            "num_obstacles": 4,
            "stage_units": get_stage_units()
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
        print(f"  Goal position: {info['goal_position']}")
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