from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():

    scene_path = os.path.join(
        os.getenv("HOME"),
        "ros2_ws/src/mujoco_multi_agv/scenes/scene.xml"
    )

    return LaunchDescription([
        Node(
            package='mujoco_multi_agv',
            executable='mujoco_sim_node',
            name='mujoco_sim_node',
            parameters=[
                {"scene_path": scene_path},
                {"num_robots": 3}
            ],
            output='screen'
        )
    ])