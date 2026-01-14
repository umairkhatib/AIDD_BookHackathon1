# Capstone Simulation Examples

This directory contains comprehensive simulation examples that demonstrate the integration of all course concepts in the Physical AI & Humanoid Robotics Course.

## Overview

The capstone simulation examples showcase:

1. **Complete System Integration**: All modules working together
2. **Vision-Language-Action (VLA) Systems**: Processing natural language commands to execute robotic actions
3. **Perception and Navigation**: Using Isaac tools for environment understanding and navigation
4. **LLM Integration**: Leveraging large language models for command interpretation

## Directory Structure

```
capstone_simulation/
├── capstone_demo.py          # Main demonstration node
├── capstone_scenarios/       # Individual demonstration scenarios
│   ├── navigation_demo.py
│   ├── perception_demo.py
│   ├── vla_demo.py
│   └── full_integration_demo.py
├── isaac_integration/        # Isaac Sim integration examples
│   ├── scene_configs/
│   ├── robot_models/
│   └── sensor_configs/
├── llm_integration/          # LLM integration examples
│   ├── whisper_integration.py
│   ├── openai_integration.py
│   └── local_llm_example.py
├── test_scenarios/           # Automated test scenarios
│   ├── basic_functionality.py
│   ├── integration_tests.py
│   └── stress_tests.py
└── README.md                 # This file
```

## Prerequisites

Before running the capstone simulations, ensure you have:

1. **ROS 2 Humble**: Properly installed and sourced
2. **Isaac Sim**: Installed and configured
3. **Isaac ROS Packages**: Installed and working
4. **OpenAI Whisper**: For speech recognition
5. **LLM Access**: API keys or local models configured

## Running the Examples

### 1. Basic Capstone Demo

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to the capstone simulation directory
cd simulation-examples/capstone_simulation

# Run the main demonstration
python3 capstone_demo.py
```

### 2. Individual Component Demos

```bash
# Run navigation demo only
python3 capstone_scenarios/navigation_demo.py

# Run perception demo only
python3 capstone_scenarios/perception_demo.py

# Run VLA (Vision-Language-Action) demo only
python3 capstone_scenarios/vla_demo.py

# Run full integration demo
python3 capstone_scenarios/full_integration_demo.py
```

### 3. Isaac Sim Integration

```bash
# Launch Isaac Sim with capstone scene
# This would be done through Isaac Sim interface or launcher
```

## Capstone Demo Features

### Navigation Component
- Path planning and execution
- Obstacle avoidance
- Multi-goal navigation
- Recovery behaviors

### Perception Component
- Object detection and recognition
- Scene understanding
- Sensor fusion
- Environmental mapping

### Language Component
- Speech-to-text conversion
- Natural language understanding
- Command interpretation
- Context awareness

### Action Component
- Task planning and execution
- Motion control
- Manipulation (simulation)
- Safety monitoring

## Configuration

The capstone simulation can be configured through:

1. **Command Line Arguments**: Pass parameters when launching
2. **Configuration Files**: YAML files in `config/` directory
3. **ROS Parameters**: Runtime parameter adjustment

### Example Configuration

```bash
# Run with custom parameters
python3 capstone_demo.py --demo-type full_integration --duration 120
```

## Testing Scenarios

### Basic Functionality Test
```bash
python3 test_scenarios/basic_functionality.py
```

### Integration Test
```bash
python3 test_scenarios/integration_tests.py
```

### Stress Test
```bash
python3 test_scenarios/stress_tests.py
```

## Isaac Sim Integration

### Scene Configuration
The capstone simulation includes several Isaac Sim scenes:

- `capstone_world.usd`: Main capstone demonstration environment
- `navigation_test.usd`: Navigation-focused environment
- `perception_test.usd`: Perception-focused environment
- `vla_test.usd`: VLA system test environment

### Robot Models
- `simple_humanoid.urdf`: Basic humanoid robot model
- `mobile_base.urdf`: Simple mobile robot base
- `manipulator_arm.urdf`: Robotic arm model

## LLM Integration

### Supported Models
- OpenAI GPT models (requires API key)
- Local LLMs (Ollama, etc.)
- NVIDIA NIM services
- Custom models

### Configuration
Set up LLM access by configuring environment variables:

```bash
export OPENAI_API_KEY="your_api_key_here"
export LLM_MODEL="gpt-4"  # or other model name
```

## Performance Metrics

The capstone simulation tracks various performance metrics:

- **Navigation Success Rate**: Percentage of successful navigation tasks
- **Perception Accuracy**: Accuracy of object detection and recognition
- **Response Time**: Time from command to action execution
- **System Resource Usage**: CPU, GPU, and memory utilization

## Troubleshooting

### Common Issues

1. **Isaac Sim Not Launching**
   - Check NVIDIA GPU drivers
   - Verify Isaac Sim installation
   - Ensure sufficient system resources

2. **LLM Connection Errors**
   - Verify API keys and network connectivity
   - Check rate limits and billing
   - Confirm LLM service availability

3. **Perception Pipeline Errors**
   - Check sensor configurations
   - Verify Isaac ROS package installation
   - Ensure proper topic connections

4. **Navigation Failures**
   - Verify map quality and localization
   - Check navigation parameters
   - Ensure proper costmap configuration

## Development Guidelines

### Adding New Capstone Features
1. Follow the existing code structure and patterns
2. Implement proper error handling and logging
3. Add appropriate tests for new functionality
4. Update documentation as needed

### Best Practices
- Use modular design principles
- Implement proper safety checks
- Include comprehensive logging
- Follow ROS 2 and Python best practices
- Document complex algorithms and decisions

## Evaluation Criteria

The capstone simulation demonstrates:

1. **System Integration**: All components working together seamlessly
2. **Robustness**: Handling of various environmental conditions
3. **Performance**: Efficient resource utilization
4. **Safety**: Proper safety mechanisms and error handling
5. **Functionality**: Successful execution of complex tasks

## Next Steps

After running these examples:

1. Modify parameters to experiment with different behaviors
2. Extend the examples with additional capabilities
3. Create your own capstone project scenarios
4. Integrate with real hardware when available

## Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [ROS 2 Navigation Documentation](https://navigation.ros.org/)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- Course materials on VLA systems