"""
Launch the holonomic MuJoCo bridge (L-carriage robots).

Usage
-----
ros2 launch swarm_mujoco_bridge holonomic.launch.py
ros2 launch swarm_mujoco_bridge holonomic.launch.py n_robots:=4 goal_x:=5.0
ros2 launch swarm_mujoco_bridge holonomic.launch.py n_robots:=4 payload_mass:=15.0

Verify
------
ros2 topic echo /swarm/payload/state
ros2 topic echo /swarm/robot_0/carriage_base_force
ros2 topic echo /swarm/robot_0/carriage_wall_force
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
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_theta', default_value='0.0'),
        DeclareLaunchArgument('payload_mass', default_value='10.0',
                              description='Payload mass in kg'),

        Node(
            package='swarm_mujoco_bridge',
            executable='holonomic_bridge_node',
            name='holonomic_mujoco_bridge',
            output='screen',
            parameters=[{
                'n_robots':          LaunchConfiguration('n_robots'),
                'scene_xml':         LaunchConfiguration('scene_xml'),
                'sim_frequency':     LaunchConfiguration('sim_frequency'),
                'control_frequency': LaunchConfiguration('control_frequency'),
                'goal_x':            LaunchConfiguration('goal_x'),
                'goal_y':            LaunchConfiguration('goal_y'),
                'goal_theta':        LaunchConfiguration('goal_theta'),
                'payload_mass':      LaunchConfiguration('payload_mass'),
            }],
        ),
    ])
