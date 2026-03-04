"""
Launch the MuJoCo bridge simulation node.

Usage:
    ros2 launch swarm_mujoco_bridge sim.launch.py
    ros2 launch swarm_mujoco_bridge sim.launch.py n_robots:=4 push_distance:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='2',
                              description='Number of robots'),
        DeclareLaunchArgument('scene_xml', default_value='',
                              description='Path to MuJoCo XML (empty = auto-generate)'),
        DeclareLaunchArgument('sim_frequency', default_value='200.0',
                              description='Physics step frequency (Hz)'),
        DeclareLaunchArgument('control_frequency', default_value='50.0',
                              description='State publish / cmd_vel apply frequency (Hz)'),
        DeclareLaunchArgument('push_distance', default_value='5.0',
                              description='Push distance for auto-generated scene (m)'),
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_theta', default_value='0.0'),

        Node(
            package='swarm_mujoco_bridge',
            executable='bridge_node',
            name='swarm_mujoco_bridge',
            output='screen',
            parameters=[{
                'n_robots': LaunchConfiguration('n_robots'),
                'scene_xml': LaunchConfiguration('scene_xml'),
                'sim_frequency': LaunchConfiguration('sim_frequency'),
                'control_frequency': LaunchConfiguration('control_frequency'),
                'push_distance': LaunchConfiguration('push_distance'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_theta': LaunchConfiguration('goal_theta'),
            }],
        ),
    ])
