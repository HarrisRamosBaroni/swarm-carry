"""
Launch the centralized controller node.

Usage:
    ros2 launch swarm_central central.launch.py n_robots:=4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='2',
                              description='Number of robots in the swarm'),
        DeclareLaunchArgument('control_frequency', default_value='50.0',
                              description='Control loop frequency (Hz)'),
        DeclareLaunchArgument('namespace', default_value='/swarm',
                              description='ROS2 topic namespace'),
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_theta', default_value='0.0'),

        Node(
            package='swarm_central',
            executable='central_node',
            name='swarm_central',
            output='screen',
            parameters=[{
                'n_robots': LaunchConfiguration('n_robots'),
                'control_frequency': LaunchConfiguration('control_frequency'),
                'namespace': LaunchConfiguration('namespace'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_theta': LaunchConfiguration('goal_theta'),
            }],
        ),
    ])
