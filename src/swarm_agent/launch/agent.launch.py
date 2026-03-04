"""
Launch a single swarm agent controller node.

Usage:
    ros2 launch swarm_agent agent.launch.py agent_id:=0 neighbor_ids:="1,2"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agent_id', default_value='0',
                              description='This robot\'s agent ID'),
        DeclareLaunchArgument('neighbor_ids', default_value='',
                              description='Comma-separated neighbor agent IDs'),
        DeclareLaunchArgument('control_frequency', default_value='50.0',
                              description='Control loop frequency (Hz)'),
        DeclareLaunchArgument('namespace', default_value='/swarm',
                              description='ROS2 topic namespace'),
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_theta', default_value='0.0'),

        Node(
            package='swarm_agent',
            executable='agent_node',
            name=['swarm_agent_', LaunchConfiguration('agent_id')],
            output='screen',
            parameters=[{
                'agent_id': LaunchConfiguration('agent_id'),
                'neighbor_ids': LaunchConfiguration('neighbor_ids'),
                'control_frequency': LaunchConfiguration('control_frequency'),
                'namespace': LaunchConfiguration('namespace'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_theta': LaunchConfiguration('goal_theta'),
            }],
        ),
    ])
