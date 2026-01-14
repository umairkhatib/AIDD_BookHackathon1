# Unity Integration Examples for Physical AI & Humanoid Robotics Course

This directory contains examples and resources for integrating Unity with the robotics simulation framework.

## Overview

Unity provides advanced rendering capabilities and Human-Robot Interaction (HRI) features that complement physics simulation in Gazebo. This section covers:

- Setting up Unity Robotics packages
- Creating robot models in Unity
- Integrating with ROS/ROS 2
- Implementing HRI scenarios

## Prerequisites

- Unity Hub and Unity Editor (2021.3 LTS or later recommended)
- Unity Robotics Package (com.unity.robotics.urp)
- ROS-TCP-Connector
- URDF Importer
- ML-Agents (optional, for AI training)

## Setup Instructions

### 1. Install Unity Robotics Packages

1. Open Unity Hub and create a new 3D project
2. Go to Window → Package Manager
3. Install the following packages:
   - Unity Robotics Package
   - ROS-TCP-Connector
   - URDF Importer

### 2. Configure ROS Connection

In your Unity project, set up the ROS-TCP-Connector:

```csharp
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;
        // Connect to ROS master
    }
}
```

### 3. Import Robot Models

Use the URDF Importer to import robot models from URDF files:

1. Place your URDF file in the Assets folder
2. Use the URDF Importer window to import the robot
3. Configure joint limits and controllers

## Example Scenes

### 1. Basic Robot Control

This scene demonstrates basic robot control through ROS integration.

### 2. HRI Environment

A human-robot interaction scenario with advanced visualization.

### 3. Training Environment

A reinforcement learning environment for robot training.

## Integration with Course Materials

The Unity examples complement the Gazebo simulation examples in the course:

- Use Gazebo for physics accuracy and sensor simulation
- Use Unity for high-fidelity visualization and HRI
- Implement co-simulation where appropriate

## Best Practices

1. **Performance**: Keep Unity scenes optimized for real-time simulation
2. **Synchronization**: Maintain consistent state between Unity and Gazebo
3. **Modularity**: Design scenes that can be easily extended
4. **Documentation**: Comment code and provide scene descriptions

## Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer)
- [Unity ML-Agents](https://github.com/Unity-Technologies/ml-agents)

## Troubleshooting

### Common Issues

1. **Connection Issues**: Ensure ROS master is running and ports are accessible
2. **Performance**: Reduce polygon count and optimize materials
3. **Synchronization**: Check time synchronization between systems

## Next Steps

Once you have Unity set up, you can:

1. Create custom robot models
2. Design HRI scenarios
3. Implement reinforcement learning environments
4. Integrate with the rest of the course simulation framework