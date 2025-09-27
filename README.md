# SO ARM101 Utils Package

This package provides utility nodes for SO ARM101 robot message conversion and adapters.

## Components

### Gripper Action Adapter

The `gripper_action_adapter` node bridges between standard ROS 2 gripper interfaces and the SO ARM101's `JointGroupPositionController`.

#### Features

- **Action Interface**: Provides `control_msgs/action/ParallelGripperCommand` action server
- **Topic Interface**: Subscribes to `control_msgs/msg/GripperCommand` messages
- **Position Control**: Converts gripper commands to normalized position values (0-1)
- **Mock Feedback**: Provides realistic action feedback for testing and integration

#### Parameters

- `action_server_name` (string, default: "gripper_cmd"): Name of the action server
- `gripper_command_topic` (string, default: "gripper_command"): Topic for simple gripper commands
- `position_controller_topic` (string, default: "/so_arm101_gripper_position_controller/commands"): Target controller topic
- `max_gripper_width` (double, default: 1.0): Maximum gripper opening value (normalized)
- `execution_duration` (double, default: 0.1): Mock execution time in seconds

#### Usage

The adapter is automatically started by the SO ARM101 bringup launch files. You can control the gripper using:

**Action Interface:**
```bash
ros2 action send_goal /gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {position: [0.5], effort: [10.0]}}"
```

**Topic Interface:**
```bash
ros2 topic pub /gripper_command control_msgs/msg/GripperCommand "{position: 0.5, max_effort: 10.0}"
```

**Direct Controller Access (Legacy):**
```bash
ros2 topic pub /so_arm101_gripper_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}"
```

#### Gripper Position Values

- `0.0`: Fully closed
- `1.0`: Fully open
- Values are normalized between 0-1

## Integration

This package is automatically included in all SO ARM101 bringup launch files:
- `so_arm101_joint_position_control.launch.py`
- `so_arm101_joint_trajectory_control.launch.py`
- `so_arm101_cartesian_motion_control.launch.py`

## Dependencies

- `rclcpp`: ROS 2 C++ client library
- `rclcpp_action`: ROS 2 action support
- `control_msgs`: Control message types
- `sensor_msgs`: Sensor message types (for JointState)
- `std_msgs`: Standard message types