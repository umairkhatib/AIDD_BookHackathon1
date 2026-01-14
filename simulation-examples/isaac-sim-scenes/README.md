# Isaac Sim Scene Examples for Physical AI & Humanoid Robotics Course

This directory contains Isaac Sim scene examples for the perception and navigation modules of the course.

## Overview

Isaac Sim provides photorealistic simulation and synthetic data generation capabilities that complement traditional physics-based simulators. These examples demonstrate how to set up scenes for perception, navigation, and AI training.

## Directory Structure

### 1. Basic Navigation (`basic_navigation/`)
Contains simple navigation scenes with:
- Basic indoor environments
- Simple obstacles
- Navigation goals and waypoints
- Sample robot configurations

### 2. Sensor Integration (`sensor_integration/`)
Demonstrates various sensor configurations:
- RGB-D cameras
- LIDAR sensors
- IMU sensors
- Multi-sensor fusion scenarios

### 3. VLA Demo (`vla_demo/`)
Examples for Vision-Language-Action integration:
- Object recognition and manipulation
- Natural language understanding
- Action planning scenarios

## Scene Setup Requirements

### Hardware Requirements
- NVIDIA GPU with CUDA support (RTX 3060 or equivalent recommended)
- 16GB+ system RAM
- 50GB+ free disk space

### Software Requirements
- Isaac Sim (based on Omniverse)
- NVIDIA GPU drivers supporting CUDA
- Compatible Python environment

## Scene Configuration

### Basic Navigation Scene
```python
# Example scene setup
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Initialize world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Add robot and configure for navigation
# Add obstacles and navigation goals
```

### Sensor Integration Scene
```python
# Example sensor configuration
from omni.isaac.sensor import Camera, RotatingLidarSensor

# Add RGB camera
rgb_camera = Camera(
    prim_path="/World/Robot/Camera",
    position=np.array([0.3, 0, 0.1]),
    frequency=30
)

# Add LIDAR sensor
lidar_sensor = RotatingLidarSensor(
    prim_path="/World/Robot/Lidar",
    translation=np.array([0.3, 0, 0.3]),
    name="RotatingLidar",
    fov=360,
    horizontal_resolution=1,
    vertical_resolution=32,
    range=10
)
```

## Isaac ROS Integration

These scenes are designed to work with Isaac ROS packages:

- **isaac_ros_detectnet**: Object detection acceleration
- **isaac_ros_segmentation**: Semantic segmentation acceleration
- **isaac_ros_stereo_rectification**: Stereo processing acceleration
- **isaac_ros_pointcloud**: Point cloud processing acceleration
- **isaac_ros_mapper**: Accelerated mapping
- **isaac_ros_goalsetter**: Navigation goal setting

## Synthetic Data Generation

Scenes can be configured for synthetic data generation:

- Photorealistic rendering
- Ground truth annotations
- Domain randomization
- Physics simulation

## Performance Considerations

### Real-time Simulation
- Adjust rendering quality based on hardware
- Limit scene complexity for real-time performance
- Use appropriate level-of-detail settings

### Training Data Generation
- Enable high-quality rendering for synthetic data
- Configure domain randomization parameters
- Set up annotation pipelines

## Best Practices

1. **Scene Optimization**: Keep scenes optimized for your use case
2. **Asset Management**: Use appropriate assets for your hardware
3. **Configuration**: Document scene configurations
4. **Testing**: Verify scenes work with your Isaac ROS pipeline

## Getting Started

1. Ensure Isaac Sim is properly installed
2. Set up the required environment variables
3. Launch Isaac Sim with one of the example scenes
4. Integrate with your ROS 2 navigation stack

## Troubleshooting

### Common Issues

1. **Rendering Issues**: Check GPU drivers and Isaac Sim compatibility
2. **Performance**: Reduce scene complexity or rendering quality
3. **ROS Integration**: Verify ROS bridge configuration
4. **Sensor Data**: Check sensor configurations and calibrations

## Next Steps

Once you have these scenes set up:
1. Integrate with your navigation stack
2. Configure for your specific robot model
3. Set up synthetic data generation pipelines
4. Implement perception and navigation algorithms