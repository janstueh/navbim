#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:

    # Get the launch directory
    bringup_dir = get_package_share_directory('navbim_bringup')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    ifc = LaunchConfiguration('ifc')
    navigation_model_filepath = LaunchConfiguration('nav_model')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description=(
            'Top-level namespace. The value will be used to replace the '
            '<robot_namespace> keyword on the rviz config file.'
        ),
    )

    declare_ifc_cmd = DeclareLaunchArgument(
        'ifc', 
        default_value='Toy_example', 
        description='IFC model name (used to construct default path for nav_model)'
    )

    declare_navigation_model_cmd = DeclareLaunchArgument(
        'nav_model', 
        default_value=[bringup_dir, '/nav_model/', ifc], 
        description='Full path to the navigation model folder (overrides ifc argument if specified)'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'navbim_default_view.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'navbim_params.yaml'),
        description='Full path to the ROS2 parameters file to use',
    )

    # Launch rviz with dynamic parameters based on nav_model
    def launch_rviz(context, *args, **kwargs):
        
        # Get configurations
        namespace_value = context.launch_configurations.get('namespace', '')
        nav_model_value = context.launch_configurations.get('nav_model', '')
        rviz_config_value = context.launch_configurations.get('rviz_config', '')
        use_sim_time_value = context.launch_configurations.get('use_sim_time', 'false').lower() == 'true'
        params_file_value = context.launch_configurations.get('params_file', '')
        
        configured_params = ParameterFile(
            params_file_value,
            allow_substs=True,
        )
        
        # Prepare ROS parameters
        parameters = [
            configured_params,
            {'use_sim_time': use_sim_time_value},
            {'nav_model': nav_model_value}
        ]
        
        start_rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace_value,
            arguments=['-d', rviz_config_value, '--ros-args', '--log-level', 'warn'],
            output='screen',
            parameters=parameters,
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )
        
        # Add exit event handler for this specific node
        exit_event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=start_rviz_cmd,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
            ),
        )
        
        return [start_rviz_cmd, exit_event_handler]
    
    start_rviz_opaque = OpaqueFunction(function=launch_rviz)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_ifc_cmd)
    ld.add_action(declare_navigation_model_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add any conditioned actions
    ld.add_action(start_rviz_opaque)

    # Add other nodes and processes we need
    # (exit event handler is now included in the OpaqueFunction return)

    return ld
