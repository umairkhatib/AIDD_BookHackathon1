#!/usr/bin/env python3

"""
VLA (Vision-Language-Action) Scene Configuration for Isaac Sim
This example demonstrates how to set up a scene for Vision-Language-Action integration.
"""

import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.isaac.sensor import Camera
from pxr import Gf, UsdGeom, Sdf
import numpy as np
import math


class VLAScene:
    def __init__(self):
        # Initialize the world
        self.world = World(stage_units_in_meters=1.0)

        # Scene parameters
        self.room_size = 10.0  # meters
        self.robot_pos = np.array([0, 0, 0.3])

        print("VLA Scene initialized")

    def setup_environment(self):
        """Set up the VLA demonstration environment."""
        print("Setting up VLA demonstration environment...")

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Create room boundaries
        self._create_room_boundaries()

        # Add objects for manipulation tasks
        self._add_manipulation_objects()

        # Add robot with vision sensors
        self._add_robot_with_vision()

        # Add human avatar for interaction
        self._add_human_avatar()

        print("VLA demonstration environment setup complete")

    def _create_room_boundaries(self):
        """Create walls with labels for VLA tasks."""
        wall_thickness = 0.1
        wall_height = 2.0

        # Calculate wall positions
        room_half = self.room_size / 2.0

        # Create walls with different materials for recognition
        walls = [
            {"name": "NorthWall", "pos": [0, room_half, wall_height/2], "rotation": [0.707, 0, 0, 0.707], "color": [0.8, 0.8, 0.8]},
            {"name": "SouthWall", "pos": [0, -room_half, wall_height/2], "rotation": [0.707, 0, 0, 0.707], "color": [0.6, 0.6, 0.6]},
            {"name": "EastWall", "pos": [room_half, 0, wall_height/2], "rotation": [0.707, 0, 0.707, 0], "color": [0.4, 0.4, 0.4]},
            {"name": "WestWall", "pos": [-room_half, 0, wall_height/2], "rotation": [0.707, 0, 0.707, 0], "color": [0.2, 0.2, 0.2]}
        ]

        for i, wall in enumerate(walls):
            create_prim(
                prim_path=f"/World/{wall['name']}",
                prim_type="Cylinder",
                position=np.array(wall['pos']),
                orientation=np.array(wall['rotation']),
                scale=np.array([wall_thickness, self.room_size, wall_height]),
                attributes={"radius": wall_thickness/2, "height": wall_height}
            )

    def _add_manipulation_objects(self):
        """Add objects for manipulation and recognition tasks."""
        print("Adding manipulation objects...")

        # Add objects with different shapes, colors, and labels
        objects = [
            {"name": "RedBall", "type": "sphere", "pos": [2, 1, 0.3], "size": 0.2, "color": [0.8, 0.2, 0.2], "label": "ball"},
            {"name": "BlueBox", "type": "cube", "pos": [-2, 1, 0.3], "size": 0.3, "color": [0.2, 0.2, 0.8], "label": "box"},
            {"name": "GreenCylinder", "type": "cylinder", "pos": [1, -2, 0.4], "size": [0.2, 0.2, 0.6], "color": [0.2, 0.8, 0.2], "label": "cylinder"},
            {"name": "YellowPyramid", "type": "pyramid", "pos": [-1, -2, 0.4], "size": 0.4, "color": [0.8, 0.8, 0.2], "label": "pyramid"},
            {"name": "LargeTable", "type": "table", "pos": [0, 0, 0.5], "size": [1.5, 1.0, 0.05], "color": [0.6, 0.4, 0.2], "label": "table"},
            {"name": "SmallBox", "type": "small_box", "pos": [0, 1.5, 0.3], "size": 0.15, "color": [0.9, 0.6, 0.1], "label": "small box"}
        ]

        for obj in objects:
            if obj["type"] == "sphere":
                self.world.scene.add(
                    DynamicSphere(
                        prim_path=f"/World/{obj['name']}",
                        name=obj['name'],
                        position=np.array(obj["pos"]),
                        radius=obj["size"],
                        color=np.array(obj["color"])
                    )
                )
            elif obj["type"] == "table":
                # Create a table with legs
                self._create_table(obj)
            else:
                # Create cuboid for other shapes
                self.world.scene.add(
                    DynamicCuboid(
                        prim_path=f"/World/{obj['name']}",
                        name=obj['name'],
                        position=np.array(obj["pos"]),
                        size=obj["size"],
                        color=np.array(obj["color"])
                    )
                )

    def _create_table(self, table_obj):
        """Create a table with legs."""
        # Table top
        self.world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/{table_obj['name']}_top",
                name=f"{table_obj['name']}_top",
                position=np.array(table_obj["pos"]),
                size=table_obj["size"],
                color=np.array(table_obj["color"])
            )
        )

        # Table legs
        leg_height = 0.5
        leg_size = 0.05
        table_pos = np.array(table_obj["pos"])
        half_x = table_obj["size"][0] / 2 - 0.1
        half_y = table_obj["size"][1] / 2 - 0.1

        leg_positions = [
            [table_pos[0] - half_x, table_pos[1] - half_y, table_pos[2] - leg_height/2],
            [table_pos[0] + half_x, table_pos[1] - half_y, table_pos[2] - leg_height/2],
            [table_pos[0] - half_x, table_pos[1] + half_y, table_pos[2] - leg_height/2],
            [table_pos[0] + half_x, table_pos[1] + half_y, table_pos[2] - leg_height/2]
        ]

        for i, leg_pos in enumerate(leg_positions):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/{table_obj['name']}_leg_{i}",
                    name=f"{table_obj['name']}_leg_{i}",
                    position=np.array(leg_pos),
                    size=leg_size,
                    color=np.array([0.3, 0.2, 0.1])
                )
            )

    def _add_robot_with_vision(self):
        """Add a robot equipped with vision sensors for VLA tasks."""
        print("Adding robot with vision sensors...")

        # Add robot base
        create_prim(
            prim_path="/World/VLARobot",
            prim_type="Cuboid",
            position=self.robot_pos,
            scale=np.array([0.5, 0.4, 0.3]),
            attributes={"size": 1.0}
        )

        # Add multiple cameras for different views
        self._add_front_camera()
        self._add_overhead_camera()
        self._add_manipulation_camera()

        print("VLA robot with cameras added successfully")

    def _add_front_camera(self):
        """Add a front-facing camera for object recognition."""
        print("Adding front camera...")

        camera_path = "/World/VLARobot/FrontCamera"
        cam_pos = self.robot_pos + np.array([0.3, 0, 0.1])  # Position in front of robot

        create_prim(
            prim_path=camera_path,
            prim_type="Camera",
            position=cam_pos,
            orientation=np.array([0, 0, 0, 1])  # Looking forward
        )

        # Configure camera properties
        stage = omni.usd.get_context().get_stage()
        camera_prim = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
        camera_prim.GetFocalLengthAttr().Set(24.0)
        camera_prim.GetHorizontalApertureAttr().Set(20.955)
        camera_prim.GetVerticalApertureAttr().Set(15.2908)

        print("Front camera configured")

    def _add_overhead_camera(self):
        """Add an overhead camera for scene understanding."""
        print("Adding overhead camera...")

        overhead_camera_path = "/World/OverheadCamera"
        overhead_pos = [0, 0, 5.0]  # Above the scene

        create_prim(
            prim_path=overhead_camera_path,
            prim_type="Camera",
            position=overhead_pos,
            orientation=np.array([0.707, 0, 0, 0.707])  # Looking down
        )

        # Configure overhead camera properties
        stage = omni.usd.get_context().get_stage()
        overhead_cam_prim = UsdGeom.Camera(stage.GetPrimAtPath(overhead_camera_path))
        overhead_cam_prim.GetFocalLengthAttr().Set(35.0)
        overhead_cam_prim.GetHorizontalApertureAttr().Set(36.0)
        overhead_cam_prim.GetVerticalApertureAttr().Set(24.0)

        print("Overhead camera configured")

    def _add_manipulation_camera(self):
        """Add a close-up camera for manipulation tasks."""
        print("Adding manipulation camera...")

        manip_camera_path = "/World/VLARobot/ManipulationCamera"
        manip_pos = self.robot_pos + np.array([0.1, 0, 0.3])  # Close to potential manipulation area

        create_prim(
            prim_path=manip_camera_path,
            prim_type="Camera",
            position=manip_pos,
            orientation=np.array([0, 0, 0, 1])  # Looking forward
        )

        # Configure manipulation camera properties
        stage = omni.usd.get_context().get_stage()
        manip_cam_prim = UsdGeom.Camera(stage.GetPrimAtPath(manip_camera_path))
        manip_cam_prim.GetFocalLengthAttr().Set(50.0)  # Telephoto for close-up
        manip_cam_prim.GetHorizontalApertureAttr().Set(20.955)
        manip_cam_prim.GetVerticalApertureAttr().Set(15.2908)

        print("Manipulation camera configured")

    def _add_human_avatar(self):
        """Add a human avatar for interaction scenarios."""
        print("Adding human avatar...")

        # Create a simple human avatar representation
        human_pos = np.array([3, 3, 0.8])  # Away from robot initially

        create_prim(
            prim_path="/World/HumanAvatar",
            prim_type="Cuboid",
            position=human_pos,
            scale=np.array([0.3, 0.3, 1.6]),
            attributes={"size": 1.0}
        )

        print("Human avatar added")

    def run_vla_simulation(self, num_steps=1000):
        """Run the VLA simulation."""
        print(f"Running VLA simulation for {num_steps} steps...")

        self.world.reset()

        for i in range(num_steps):
            self.world.step(render=True)

            # Print progress periodically
            if i % 200 == 0:
                print(f"VLA simulation step: {i}")

                # In a real VLA implementation, you would:
                # - Process visual input for object recognition
                # - Interpret natural language commands
                # - Plan appropriate actions
                # - Execute manipulation tasks
                # - Monitor human-robot interaction

        print("VLA simulation completed!")

    def get_vla_config(self):
        """Get information about the VLA configuration."""
        config = {
            "cameras": ["front", "overhead", "manipulation"],
            "objects": ["ball", "box", "cylinder", "pyramid", "table", "small box"],
            "robot_position": self.robot_pos.tolist(),
            "human_interaction": True
        }
        return config


def main():
    """Main function to create and run the VLA scene."""
    print("Creating VLA Scene...")

    scene = VLAScene()

    try:
        # Set up the environment
        scene.setup_environment()

        # Print VLA configuration
        config = scene.get_vla_config()
        print(f"\nVLA Configuration:")
        print(f"  Cameras: {config['cameras']}")
        print(f"  Objects: {config['objects']}")
        print(f"  Robot position: {config['robot_position']}")
        print(f"  Human interaction: {config['human_interaction']}")

        # Run the VLA simulation
        scene.run_vla_simulation()

    except Exception as e:
        print(f"Error during VLA simulation: {e}")
        carb.log_error(str(e))
    finally:
        # Cleanup
        scene.world.clear()
        print("\nScene cleared and resources released")


if __name__ == "__main__":
    main()