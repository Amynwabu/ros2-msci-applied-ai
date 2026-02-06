# Lesson 06: Launch Files - Starting Multiple Nodes

## Purpose

Launch files allow you to **start multiple ROS2 nodes with a single command** instead of opening separate terminals.

## Creating a File 

### Step 1: Create launch directory

mkdir -p ~/ros2_ws/src/test2_py_pkg/launch

### Step 2: Create launch file
```
Navigate to: cd ~/ros2_ws/src/test2_py_pkg/launch

Create file: restaurant_system.launch.py

nano ~/ros2_ws/src/test2_py_pkg/launch/all_nodes.py
```




## CMakeLists.txt Configuration

```cmake
find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

## Running Launch File

```bash
# Build
colcon build --packages-select my_package
source install/setup.bash

# Run launch file
ros2 launch my_package my_system.launch.py
```

## Features

- **Parameters**: Pass arguments to nodes
- **Conditionals**: Run nodes conditionally
- **Namespacing**: Organize nodes in groups
- **Remapping**: Rename topics and services

## Example with Parameters

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    frequency = LaunchConfiguration('frequency', default='10')
    
    return LaunchDescription([
        DeclareLaunchArgument('frequency', default_value='10'),
        Node(
            package='my_pkg',
            executable='publisher',
            parameters=[{'frequency': frequency}]
        ),
    ])
```

## Key Benefits

✓ **Single Command**: Start entire system with one command
✓ **Repeatable**: Same configuration every time
✓ **Configurable**: Pass parameters easily
✓ **Professional**: Used in real robotics projects

## Next: Lesson 07 - Smart Restaurant System Project

Apply all skills to build a complete integrated system!
