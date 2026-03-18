from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('swarm_mocap')

    return LaunchDescription([
        DeclareLaunchArgument('server_ip',           default_value='192.168.1.71'),
        DeclareLaunchArgument('frame_id',            default_value='mocap'),
        DeclareLaunchArgument('published_rigid_ids', default_value='[]'),

        Node(
            package='swarm_mocap',
            executable='mocap_node',
            name='swarm_mocap',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'mocap_params.yaml'),
                {
                    'server_ip':           LaunchConfiguration('server_ip'),
                    'frame_id':            LaunchConfiguration('frame_id'),
                    'published_rigid_ids': LaunchConfiguration('published_rigid_ids'),
                },
            ],
        ),
    ])
