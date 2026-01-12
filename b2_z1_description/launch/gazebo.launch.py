#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file to simulate B2+Z1 combined robot in Gazebo.

This launch file:
1. Starts Gazebo with a world
2. Spawns the B2+Z1 combined robot
3. Starts necessary controllers
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    # Setup environment variables for Gazebo Classic
    GAZEBO_MODEL_PATH = "GAZEBO_MODEL_PATH"
    for pkg in ['b2_z1_description', 'b2_description', 'z1_description']:
        try:
            pkg_path = os.path.join(get_package_prefix(pkg), "share")
            if GAZEBO_MODEL_PATH in os.environ:
                if pkg_path not in os.environ[GAZEBO_MODEL_PATH]:
                    os.environ[GAZEBO_MODEL_PATH] += ":" + pkg_path
            else:
                os.environ[GAZEBO_MODEL_PATH] = pkg_path
        except:
            pass

    # Add custom gazebo_ros2_control path
    GAZEBO_PLUGIN_PATH = "GAZEBO_PLUGIN_PATH"
    custom_gazebo_ros2_control = "/home/yeti/gazebo_ros2_control/install/gazebo_ros2_control/lib"
    
    if GAZEBO_PLUGIN_PATH in os.environ:
        if custom_gazebo_ros2_control not in os.environ[GAZEBO_PLUGIN_PATH]:
            os.environ[GAZEBO_PLUGIN_PATH] = custom_gazebo_ros2_control + ":" + os.environ[GAZEBO_PLUGIN_PATH]
        if "/opt/ros/humble/lib" not in os.environ[GAZEBO_PLUGIN_PATH]:
            os.environ[GAZEBO_PLUGIN_PATH] += ":/opt/ros/humble/lib"
    else:
        os.environ[GAZEBO_PLUGIN_PATH] = custom_gazebo_ros2_control + ":/opt/ros/humble/lib"
    
    # Also set LD_LIBRARY_PATH for runtime linking
    LD_LIBRARY_PATH = "LD_LIBRARY_PATH"
    if LD_LIBRARY_PATH in os.environ:
        if custom_gazebo_ros2_control not in os.environ[LD_LIBRARY_PATH]:
            os.environ[LD_LIBRARY_PATH] = custom_gazebo_ros2_control + ":" + os.environ[LD_LIBRARY_PATH]
    else:
        os.environ[LD_LIBRARY_PATH] = custom_gazebo_ros2_control
    
    # Launch arguments
    world_name = LaunchConfiguration('world')
    z1_mount_x = LaunchConfiguration('z1_mount_x')
    z1_mount_y = LaunchConfiguration('z1_mount_y')
    z1_mount_z = LaunchConfiguration('z1_mount_z')
    z1_mount_yaw = LaunchConfiguration('z1_mount_yaw')
    
    # Declare launch arguments
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World file name (without .world extension)'
    )
    
    declare_z1_mount_x = DeclareLaunchArgument(
        'z1_mount_x',
        default_value='0.05',
        description='X position of Z1 mount on B2'
    )
    
    declare_z1_mount_y = DeclareLaunchArgument(
        'z1_mount_y',
        default_value='0.0',
        description='Y position of Z1 mount on B2'
    )
    
    declare_z1_mount_z = DeclareLaunchArgument(
        'z1_mount_z',
        default_value='0.15',
        description='Z position of Z1 mount on B2'
    )
    
    declare_z1_mount_yaw = DeclareLaunchArgument(
        'z1_mount_yaw',
        default_value='0.0',
        description='Yaw rotation of Z1 mount on B2'
    )
    
    # Path to xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('b2_z1_description'),
        'urdf',
        'b2_z1.urdf.xacro'
    ])
    
    # Controller configuration file - use robot_control_ros2.yaml for RL control
    controller_config = os.path.join(
        get_package_share_directory('b2_z1_description'),
        'config',
        'robot_control_ros2.yaml'
    )
    
    # Process xacro to generate URDF
    robot_description = Command([
        'xacro ', xacro_file,
        ' z1_mount_x:=', z1_mount_x,
        ' z1_mount_y:=', z1_mount_y,
        ' z1_mount_z:=', z1_mount_z,
        ' z1_mount_yaw:=', z1_mount_yaw,
        ' controllers:=\"', controller_config, '\"',
    ])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True
        }]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('rl_sar'),
                'worlds',
                [world_name, '.world']
            ]),
            'verbose': 'true',
            'pause': 'false',
        }.items(),
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'b2_z1_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
        output='screen',
    )
    
    # Joint state broadcaster (for all joints)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60', 
                   '--param-file', controller_config],
        output='screen',
    )
    
    # Robot joint controller (for RL control - handles all B2 leg joints + Z1 arm)
    robot_joint_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot_joint_controller', '--controller-manager-timeout', '60',
                   '--param-file', controller_config],
        output='screen',
    )
    
    return LaunchDescription([
        declare_world,
        declare_z1_mount_x,
        declare_z1_mount_y,
        declare_z1_mount_z,
        declare_z1_mount_yaw,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity,
        joint_state_broadcaster,
        robot_joint_controller,
    ])
