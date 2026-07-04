#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:

    bringup_dir = get_package_share_directory('navbim_bringup')

    namespace = LaunchConfiguration('namespace')
    ifc = LaunchConfiguration('ifc')
    ifc_file = LaunchConfiguration('ifc_file')
    navigation_model_filepath = LaunchConfiguration('nav_model')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}
    
    # When namespace is not provided, use node name directly for parameter lookup
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,  # Use namespace if provided, otherwise empty
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_ifc_cmd = DeclareLaunchArgument(
        'ifc', 
        default_value='Toy_example', 
        description='IFC model name (used to construct default paths for ifc_file and nav_model)'
    )

    declare_ifc_file_cmd = DeclareLaunchArgument(
        'ifc_file', 
        default_value=[bringup_dir, '/bim/', ifc, '.ifc'], 
        description='Full path to the IFC file (overrides ifc argument if specified)'
    )

    declare_navigation_model_cmd = DeclareLaunchArgument(
        'nav_model', 
        default_value=[bringup_dir, '/nav_model/', ifc], 
        description='Full path to the navigation model folder (overrides ifc argument if specified)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'navbim_params.yaml'),
        description='Full path to the ROS2 parameters file to use',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # Start nav model generator node
    nav_model_generator_node = Node(
        package='navbim_nav_model_gen',
        executable='nav_model_generator_node',
        name='nav_model_gen',
        namespace=namespace,
        output='screen',
        #prefix='gnome-terminal --wait --',  # Run in separate GNOME terminal for interactive visualizations
        parameters=[params_file,  # Load full params file directly
                    {'ifc_file': ifc_file},
                    {'nav_model': navigation_model_filepath}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )
    
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_ifc_cmd)
    ld.add_action(declare_ifc_file_cmd)
    ld.add_action(declare_navigation_model_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(nav_model_generator_node)

    return ld