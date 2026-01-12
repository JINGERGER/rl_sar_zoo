#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file to visualize B2+Z1 combined robot in RViz.

This launch file:
1. Processes the xacro file to generate URDF
2. Starts robot_state_publisher
3. Starts joint_state_publisher_gui for manual joint control
4. Launches RViz with a custom configuration
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_b2_z1_description = get_package_share_directory('b2_z1_description')
    
    # Launch arguments
    z1_mount_x = LaunchConfiguration('z1_mount_x')
    z1_mount_y = LaunchConfiguration('z1_mount_y')
    z1_mount_z = LaunchConfiguration('z1_mount_z')
    z1_mount_yaw = LaunchConfiguration('z1_mount_yaw')
    use_gui = LaunchConfiguration('use_gui')
    
    # Declare launch arguments
    declare_z1_mount_x = DeclareLaunchArgument(
        'z1_mount_x',
        default_value='0.18',
        description='X position of Z1 mount on B2 (forward offset in meters)'
    )
    
    declare_z1_mount_y = DeclareLaunchArgument(
        'z1_mount_y',
        default_value='0.0',
        description='Y position of Z1 mount on B2 (lateral offset in meters)'
    )
    
    declare_z1_mount_z = DeclareLaunchArgument(
        'z1_mount_z',
        default_value='0.06',
        description='Z position of Z1 mount on B2 (vertical offset in meters)'
    )
    
    declare_z1_mount_yaw = DeclareLaunchArgument(
        'z1_mount_yaw',
        default_value='0.0',
        description='Yaw rotation of Z1 mount on B2 (in radians)'
    )
    
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui for manual control'
    )
    
    # Path to xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('b2_z1_description'),
        'urdf',
        'b2_z1.urdf.xacro'
    ])
    
    # Process xacro to generate URDF
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' z1_mount_x:=', z1_mount_x,
            ' z1_mount_y:=', z1_mount_y,
            ' z1_mount_z:=', z1_mount_z,
            ' z1_mount_yaw:=', z1_mount_yaw,
        ]),
        value_type=str
    )
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher GUI node (for manual joint control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    )
    
    # Joint state publisher node (without GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui)
    )
    
    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('b2_z1_description'),
        'config',
        'view_robot.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    
    return LaunchDescription([
        declare_z1_mount_x,
        declare_z1_mount_y,
        declare_z1_mount_z,
        declare_z1_mount_yaw,
        declare_use_gui,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])
