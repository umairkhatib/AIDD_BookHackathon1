#!/bin/bash

# setup-dev-env.sh - Environment setup script for Physical AI & Humanoid Robotics Course
# This script sets up the development environment for the course, including:
# - ROS 2 Humble Hawksbill installation
# - Gazebo simulation environment
# - Docusaurus documentation site
# - Python dependencies
# - Node.js dependencies

set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Course"
echo "Environment Setup Script"
echo "==========================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to pause and wait for user input
pause() {
    read -p "Press Enter to continue..."
}

# Check for prerequisites
echo "Checking prerequisites..."

if ! command_exists git; then
    echo "ERROR: git is not installed. Please install git first."
    exit 1
fi

if ! command_exists node; then
    echo "WARNING: node is not installed. Installing Node.js..."
    echo "Please install Node.js 18.x or higher from https://nodejs.org/"
    pause
fi

if ! command_exists python3; then
    echo "ERROR: python3 is not installed. Please install Python 3.11 or higher."
    exit 1
fi

echo "Prerequisites check completed."

# Install Python dependencies
echo "Installing Python dependencies..."
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt
else
    echo "Creating requirements.txt with basic ROS 2 dependencies..."
    cat > requirements.txt << EOF
# ROS 2 Humble Hawksbill dependencies
rclpy>=3.0.0
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
control_msgs
tf2_ros
cv_bridge
message_filters
launch
launch_ros
ros2launch
rosbags
numpy
scipy
matplotlib
pandas
opencv-python
EOF
    pip3 install -r requirements.txt
fi

# Install Node.js dependencies
echo "Installing Node.js dependencies..."
cd website
if [ -f "package.json" ]; then
    npm install
else
    echo "Creating basic package.json for Docusaurus..."
    npm init -y
    npm install @docusaurus/core @docusaurus/preset-classic clsx prism-react-renderer
fi

# Check if ROS 2 is installed
if command_exists ros2; then
    echo "ROS 2 is already installed."
else
    echo "ROS 2 Humble Hawksbill is not installed."
    echo "Please follow the official ROS 2 Humble installation guide:"
    echo "https://docs.ros.org/en/humble/Installation.html"
    echo ""
    echo "For Ubuntu 22.04 LTS:"
    echo "sudo apt update && sudo apt install curl gnupg lsb-release"
    echo "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-iron.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
    echo "echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
    echo "sudo apt update"
    echo "sudo apt install ros-humble-desktop"
    echo "sudo apt install python3-colcon-common-extensions"
    echo "sudo apt install python3-rosdep"
    echo "sudo rosdep init"
    echo "rosdep update"
    echo ""
    echo "After installing ROS 2, remember to source the environment:"
    echo "source /opt/ros/humble/setup.bash"
    pause
fi

# Check if Gazebo is installed
if command_exists ign; then
    echo "Gazebo/Ignition is installed."
elif command_exists gazebo; then
    echo "Gazebo Classic is installed."
else
    echo "Gazebo simulation environment is not installed."
    echo "Installing Gazebo Garden (recommended for ROS 2 Humble):"
    echo "sudo apt install ros-humble-gazebo-* ros-humble-ign*"
    pause
fi

# Create a basic ROS 2 workspace if it doesn't exist
if [ ! -d "simulation-examples/ros2-workspaces/basic_nodes" ]; then
    echo "Creating basic ROS 2 workspace structure..."
    mkdir -p simulation-examples/ros2-workspaces/basic_nodes
    cat > simulation-examples/ros2-workspaces/basic_nodes/simple_publisher.py << 'EOF'
#!/usr/bin/env python3

"""
Simple ROS 2 publisher example for the Physical AI & Humanoid Robotics Course
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    rclpy.spin(simple_publisher)

    # Destroy the node explicitly
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
fi

echo "Environment setup completed!"
echo ""
echo "To start the documentation site:"
echo "cd website && npm run start"
echo ""
echo "To run ROS 2 examples:"
echo "source /opt/ros/humble/setup.bash"
echo "cd simulation-examples/ros2-workspaces && python3 basic_nodes/simple_publisher.py"
echo ""
echo "Happy learning with Physical AI & Humanoid Robotics!"