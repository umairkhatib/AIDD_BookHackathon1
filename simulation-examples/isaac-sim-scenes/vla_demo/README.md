# Vision-Language-Action (VLA) Demo Examples

This directory contains Vision-Language-Action (VLA) demonstration examples for the Physical AI & Humanoid Robotics Course. These examples showcase how to integrate visual perception, language understanding, and action execution in Isaac Sim.

## Overview

The VLA demo examples demonstrate:

1. **Visual Perception**: Using Isaac Sim sensors for environmental understanding
2. **Language Processing**: Integrating LLMs with robotic perception
3. **Action Execution**: Converting language commands to robot actions
4. **Real-time Integration**: Coordinating all components in real-time

## Directory Structure

```
vla_demo/
├── basic_navigation/
│   ├── scene_config.py          # Basic navigation scene configuration
│   ├── voice_navigation.py      # Voice-controlled navigation example
│   └── README.md               # Navigation demo documentation
├── object_interaction/
│   ├── scene_config.py          # Object interaction scene configuration
│   ├── grasp_demo.py           # Grasping demonstration
│   └── README.md               # Object interaction demo documentation
├── multi_modal_task/
│   ├── scene_config.py          # Multi-modal task scene configuration
│   ├── complex_task_demo.py    # Complex task execution example
│   └── README.md               # Multi-modal task demo documentation
└── integration_test/
    ├── full_vla_demo.py        # Complete VLA integration test
    └── performance_benchmarks.py # Performance benchmarking
```

## Prerequisites

Before running these examples, ensure you have:

- Isaac Sim properly installed and configured
- ROS 2 Humble Hawksbill
- Isaac ROS packages installed
- OpenAI Whisper for speech recognition
- Large Language Model access (OpenAI API key or local LLM)

## Getting Started

### 1. Basic Navigation Demo

The basic navigation demo shows how to process voice commands to navigate to locations:

```bash
cd basic_navigation
python3 voice_navigation.py
```

In a separate terminal, you can test with:
```bash
# Use a voice command or simulate one
rosservice call /voice_command "command: 'Go to the kitchen'"
```

### 2. Object Interaction Demo

The object interaction demo demonstrates how to process language commands to manipulate objects:

```bash
cd object_interaction
python3 grasp_demo.py
```

### 3. Multi-Modal Task Demo

The multi-modal task demo combines perception, language, and action for complex tasks:

```bash
cd multi_modal_task
python3 complex_task_demo.py
```

## VLA Integration Components

### Perception Pipeline

The perception pipeline integrates:

- RGB-D cameras for visual input
- LIDAR for spatial understanding
- IMU for orientation data
- Force/torque sensors for manipulation feedback

### Language Processing

The language processing component:

- Uses Whisper for speech-to-text conversion
- Integrates with LLMs for command interpretation
- Maps natural language to action sequences
- Handles contextual understanding

### Action Execution

The action execution system:

- Translates high-level commands to low-level actions
- Coordinates multiple robot subsystems
- Monitors execution and handles errors
- Provides feedback to the user

## Example Scenes

### 1. Kitchen Assistant Scene

A scene with kitchen objects and navigation waypoints:

- Locations: Counter, stove, refrigerator, sink
- Objects: Cup, plate, knife, apple
- Tasks: "Bring me a cup from the counter", "Go to the refrigerator"

### 2. Office Assistant Scene

A scene with office objects and navigation waypoints:

- Locations: Desk, printer, filing cabinet, meeting room
- Objects: Pen, paper, stapler, laptop
- Tasks: "Find the pen on the desk", "Go to the printer"

### 3. Living Room Assistant Scene

A scene with living room objects and navigation waypoints:

- Locations: Sofa, TV, coffee table, bookshelf
- Objects: Remote, book, blanket, glass
- Tasks: "Get the remote from the coffee table", "Follow me to the sofa"

## Performance Considerations

### Real-time Requirements

- Perception pipeline: < 50ms processing time
- Language understanding: < 1000ms response time
- Action execution: < 10ms command latency
- Overall system: < 2000ms from command to action initiation

### Resource Management

- GPU memory: Ensure sufficient VRAM for Isaac Sim and neural networks
- CPU resources: Multi-threading for perception and action execution
- Network: Low-latency communication for LLM integration
- Storage: Efficient data streaming for continuous operation

## Troubleshooting

### Common Issues

1. **Isaac Sim Not Launching**
   - Check NVIDIA GPU drivers and CUDA installation
   - Verify Isaac Sim installation path
   - Ensure sufficient system resources

2. **Whisper Model Loading Issues**
   - Check internet connection for model download
   - Verify Python environment and dependencies
   - Ensure sufficient disk space for model files

3. **LLM API Connection Issues**
   - Verify API key and network connectivity
   - Check rate limits and billing
   - Confirm LLM service availability

4. **ROS 2 Communication Issues**
   - Verify ROS_DOMAIN_ID consistency
   - Check network configuration for multi-machine setups
   - Confirm topic and service availability

## Customization

To create your own VLA scenes:

1. Create a new directory with your scene name
2. Implement the scene configuration in `scene_config.py`
3. Create the demo logic in your main script
4. Add documentation in a README.md file
5. Test with various language commands
6. Benchmark performance and optimize as needed

## Best Practices

1. **Error Handling**: Implement robust error handling for all components
2. **Safety**: Always include safety checks before action execution
3. **User Feedback**: Provide clear feedback on system state
4. **Context Awareness**: Consider environmental context in command interpretation
5. **Performance**: Optimize for real-time operation requirements

## Next Steps

After mastering these VLA demos, you can:

1. Integrate with real hardware
2. Expand to more complex tasks
3. Implement learning from demonstration
4. Add multi-robot coordination
5. Enhance with multimodal perception