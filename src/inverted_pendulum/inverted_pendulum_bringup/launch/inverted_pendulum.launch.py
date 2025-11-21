# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from launch.events import Shutdown
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('inverted_pendulum_bringup')
    pkg_project_gazebo = get_package_share_directory('inverted_pendulum_gazebo')
    pkg_project_description = get_package_share_directory('inverted_pendulum_description')
    pkg_project_controller = get_package_share_directory('inverted_pendulum_controller')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # # Load the SDF file from "description" package
    # sdf_file  =  os.path.join(pkg_project_description, 'models', 'inverted_pendulum', 'model.sdf')
    # # sdf_file  =  os.path.join(pkg_project_description, 'models', 'inverted_pendulum', 'urdf', 'model.urdf')
    # # sdf_file  =  os.path.join(pkg_project_description, 'models', 'inverted_pendulum', 'model.xacro.sdf')
    # with open(sdf_file, 'r') as infp:
    #     robot_desc = infp.read()
    
    # Load the URDF file
    urdf_file  =  os.path.join(pkg_project_description, 'models', 'inverted_pendulum', 'urdf', 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    print("robot_desc", robot_desc)
    # robot_desc_dict = {'robot_description': urdf_file}

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'inverted_pendulum_world.sdf'
        ])}.items(),
        # on_exit=EmitEvent(event=Shutdown())  # terminate gazebo sim and all associated nodes on gz gui shutdown
    )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', 
                   '-name', 'inverted_pendulum', 
                   '-allow_renaming', 'true'],
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # output='both',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
        # parameters=[
        #     {'use_sim_time': True},
        #     {'robot_description': robot_desc},
        # ]
    )

    # ---------------------------
    # ROS2 CONTROL stuff
    # ---------------------------
    # LQR controller node
    inverted_pendulum_controller_node = Node(
        package='inverted_pendulum_controller',
        executable='inverted_pendulum_controller',
        name='inverted_pendulum_controller',
        # remappings=[
            # ('/cart_joint/cmd_force', '/cart_effort_controller/commands')
        # ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": True},
            os.path.join(pkg_project_controller, "config", "cart_controllers.yaml"),
            {"robot_description": robot_desc},
        ],
        output="screen",
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('inverted_pendulum_controller'),
            'config',
            'cart_controllers.yaml',
        ]
    )

    robot_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["robot_joint_state_broadcaster",
                   "--controller-manager", "/controller_manager", 
                   "--switch-timeout", "20", 
                   '--param-file', robot_controllers],
    )
    
    cart_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["cart_effort_controller", 
                   "--controller-manager", "/controller_manager", 
                   "--switch-timeout", "20", 
                   '--param-file', robot_controllers,],
    )
    
    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'inverted_pendulum.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(pkg_project_bringup, 'config', 'inverted_pendulum_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        # SetParameter(name='robot_description', value=robot_desc),
        # SetParameter(name='use_sim_time', value=True),
        robot_state_publisher,
        gz_sim,  
        bridge,
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=robot_state_publisher,
        #         on_exit=[gz_spawn_entity],
        #     )
        # ),
        gz_spawn_entity,
        # controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[robot_joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_joint_state_broadcaster_spawner,
                on_exit=[cart_effort_controller_spawner],
            )
        ),
        # TimerAction(
        #     period=10.0,
        #     actions=[
        #         robot_joint_state_broadcaster_spawner,
        #         cart_effort_controller_spawner,
        #     ]
        # ),
        TimerAction(
            period=2.0,
            actions=[
                inverted_pendulum_controller_node,
            ]
        ),
        # inverted_pendulum_controller_node,
        # robot_joint_state_broadcaster_spawner,
        # cart_effort_controller_spawner,
        # DeclareLaunchArgument('rviz', default_value='true',
        #                       description='Open RViz.'),
        # DeclareLaunchArgument(
        #     'description_format',
        #     default_value='sdf',
        #     description='Robot description format to use, urdf or sdf'),
    ])
        # controller_manager_node,
        # rviz
