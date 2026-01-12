#!/usr/bin/env python3
"""
Test launch file to verify Z1 arm works individually with ros2_control in Gazebo
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_b2_z1_description = get_package_share_directory('b2_z1_description')
    
    # Paths
    world_file = PathJoinSubstitution([
        FindPackageShare('b2_z1_description'),
        'worlds',
        'empty.world'
    ])
    
    # Create a simple Z1-only URDF using xacro
    z1_urdf_path = os.path.join(pkg_b2_z1_description, 'urdf', 'test_z1_only.urdf.xacro')
    robot_description = Command(['xacro ', z1_urdf_path])
    
    # Controller config
    controller_config = os.path.join(
        pkg_b2_z1_description,
        'config',
        'z1_only_control.yaml'
    )
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    
    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ])
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'z1_test',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Z1 controller spawner
    z1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['z1_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        z1_controller_spawner,
    ])
