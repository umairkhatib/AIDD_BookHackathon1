# Quickstart Guide: Physical AI & Humanoid Robotics Course

**Feature**: 001-physical-ai-humanoid
**Date**: 2026-01-09
**Status**: Completed

## Overview

This guide provides a quick setup and introduction to the Physical AI & Humanoid Robotics course. It covers the essential steps to get started with the course materials, development environment, and simulation tools.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 50GB free space minimum
- **GPU**: NVIDIA GPU with CUDA support (RTX 3060 or equivalent recommended)
- **CPU**: Multi-core processor (Intel i7 or AMD Ryzen 7 equivalent)

### Software Dependencies
- Git version control system
- Docker and Docker Compose
- Python 3.11 or higher
- Node.js 18.x or higher
- npm/yarn package managers

## Installation Steps

### 1. Clone the Repository
```bash
git clone https://github.com/[organization]/physical-ai-humanoid-course.git
cd physical-ai-humanoid-course
```

### 2. Set Up Development Environment
```bash
# Navigate to the project root
cd physical-ai-humanoid-course

# Run the environment setup script
./scripts/setup-dev-env.sh

# Or for manual setup:
# Install Python dependencies
pip3 install -r requirements.txt

# Install Node.js dependencies
cd website
npm install
```

### 3. Install ROS 2 (Humble Hawksbill)
Follow the official ROS 2 Humble installation guide:
```bash
# Add ROS 2 repository and keys
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-iron.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### 4. Install Simulation Tools
```bash
# Install Gazebo Garden
sudo apt install ros-humble-gazebo-* ros-humble-ign*

# For NVIDIA Isaac Sim (follow official installation guide)
# Download from NVIDIA Developer portal and follow installation instructions
```

### 5. Verify Installation
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Test ROS 2 installation
ros2 topic list

# Test Gazebo installation
ign gazebo -v
```

## Course Navigation

### Website Structure
The course is organized into 4 main modules:

```
website/docs/
├── intro/                    # Introduction and prerequisites
├── module-1-ros-foundations/ # ROS 2 fundamentals
├── module-2-digital-twin/    # Simulation environments
├── module-3-ai-brain/        # Perception and navigation
├── module-4-vla/             # Vision-Language-Action systems
└── capstone-project/         # Comprehensive project
```

### Starting the Course Website
```bash
# Navigate to website directory
cd website

# Start the development server
npm run start

# The course will be available at http://localhost:3000
```

## First Exercise: Hello ROS 2

### Objective
Create your first ROS 2 node that publishes messages to control a simulated joint.

### Steps
1. Navigate to the ROS 2 workspace:
```bash
cd simulation-examples/ros2-workspaces/basic_nodes
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

2. Run the example publisher node:
```bash
ros2 run demo_nodes_py talker
```

3. In another terminal, run the subscriber:
```bash
ros2 run demo_nodes_py listener
```

### Expected Output
You should see messages being published from the talker node and received by the listener node, demonstrating basic ROS 2 communication.

## Simulation Environment Setup

### Running Your First Simulation
```bash
# Navigate to Gazebo examples
cd simulation-examples/gazebo_worlds

# Launch a basic world
ign gazebo basic_world.sdf
```

### Loading a Humanoid Robot
```bash
# Launch Gazebo with a humanoid model
ros2 launch simulation_examples bringup.launch.py
```

## Development Workflow

### 1. Course Material Development
```bash
# Edit course content in the docs directory
website/docs/module-1-ros-foundations/concepts/ros2-architecture.mdx

# The website will automatically reload with changes
```

### 2. Simulation Development
```bash
# Modify simulation configurations
simulation-examples/ros2-workspaces/urdf_models/humanoid.urdf

# Test changes with:
ign gazebo -r -v 4 humanoid_model.sdf
```

### 3. Testing Your Changes
```bash
# Run course content tests
npm run test

# Run simulation tests
cd simulation-examples
pytest test_simulation.py
```

## Troubleshooting

### Common Issues

#### ROS 2 Installation Problems
- **Issue**: Package not found during ROS 2 installation
- **Solution**: Ensure your Ubuntu version matches the ROS 2 distribution requirements

#### GPU/CUDA Issues
- **Issue**: Simulation runs slowly or crashes
- **Solution**: Verify CUDA drivers are properly installed and compatible with your GPU

#### Network/Port Issues
- **Issue**: ROS 2 nodes can't communicate across machines
- **Solution**: Check network configuration and firewall settings

### Getting Help
- Check the FAQ section in the course documentation
- Visit the discussion forum for the course
- Review the troubleshooting section of each module

## Next Steps

1. Complete the introductory module on course setup and prerequisites
2. Proceed to Module 1: ROS 2 Foundations to learn about robot middleware
3. Practice with the provided exercises and examples
4. Move on to simulation environments in Module 2

## Useful Commands

```bash
# Start the course website
cd website && npm start

# Build ROS 2 packages
cd simulation-examples/ros2-workspaces && colcon build

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# List available ROS 2 topics
ros2 topic list

# Run simulation examples
cd simulation-examples && python3 run_example.py
```

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim Documentation](https://docs.nvidia.com/isaac/)
- [Docusaurus Documentation](https://docusaurus.io/docs)