

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test2_py_pkg',
            executable='my_first_node',
            name='my_first_node',
            output='screen',
        ),
        Node(
            package='test2_py_pkg',
            executable='create_topic',
            name='create_topic',
            output='screen',
        ),
        Node(
            package='test2_py_pkg',
            executable='waiter',
            name='waiter',
            output='screen',
        ),
        Node(
            package='test2_py_pkg',
            executable='chef',
            name='chef',
            output='screen',
        ),
        Node(
            package='test2_py_pkg',
            executable='restaurant_server',
            name='restaurant_server',
            output='screen',
        ),
        Node(
            package='test2_py_pkg',
            executable='restaurant_client',
            name='restaurant_client',
            output='screen',
            # Example args; you can customize or pass from command line
            arguments=['burger', '2'],
        ),
    ])

