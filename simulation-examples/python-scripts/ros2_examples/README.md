# Hello World Example for Physical AI & Humanoid Robotics Course

This is a simple verification that your development environment is set up correctly.

## Running the Example

1. Make sure you have sourced your ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash  # On Linux
   # Or run this in each new terminal as needed
   ```

2. Navigate to the example directory:
   ```bash
   cd simulation-examples/python-scripts/ros2_examples
   ```

3. Run the hello world example:
   ```bash
   python3 hello_world.py
   ```

## Expected Output

You should see messages being published to the "hello_world" topic:

```
[INFO] [1234567890.123456]: Publishing: 'Hello World 0'
[INFO] [1234567890.623456]: Publishing: 'Hello World 1'
[INFO] [1234567890.123456]: Publishing: 'Hello World 2'
...
```

If you see this output, your environment is correctly configured!

## Troubleshooting

- If you get "ModuleNotFoundError: No module named 'rclpy'", make sure ROS 2 is installed and sourced
- If you get permission errors, make sure the script is executable: `chmod +x hello_world.py`
- If ROS 2 commands are not found, ensure you've properly sourced the ROS 2 environment