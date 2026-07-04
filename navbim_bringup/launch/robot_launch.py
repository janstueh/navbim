#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetParameter


def generate_launch_description():
    """Generate launch description for robot setup."""
    
    description_dir = get_package_share_directory('navbim_description')
    
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='IDOG',
        description='Name of the robot (identifies which URDF to load)'
    )
    
    # Robot setup group
    robot_setup_group = GroupAction([
        PushROSNamespace(namespace=namespace),
        SetParameter('use_sim_time', use_sim_time),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command([
                    'xacro ', description_dir, '/urdf/', robot_name, '.urdf.xacro'
                ])
            }]
        ),
        # Future: Add sensor drivers, joint_state_publisher, etc. here
    ])
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    
    # Add robot setup
    ld.add_action(robot_setup_group)
    
    return ld
