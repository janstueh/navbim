#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable, OpaqueFunction, ExecuteProcess, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory('navbim_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    ifc = LaunchConfiguration('ifc')
    ifc_file = LaunchConfiguration('ifc_file')
    navigation_model_filepath = LaunchConfiguration('nav_model')
    force_nav_model_gen = LaunchConfiguration('force_nav_model_gen')
    robot_name = LaunchConfiguration('robot_name')
    gpp_bim = LaunchConfiguration('gpp_bim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    bim_alignment = LaunchConfiguration('bim_alignment')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
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

    declare_force_nav_model_gen_cmd = DeclareLaunchArgument(
        'force_nav_model_gen', default_value='False', 
        description='Whether to force the regeneration of the navigation model'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='IDOG',
        description='Name of the robot (identifies which URDF to load)'
    )


    declare_gpp_bim_cmd = DeclareLaunchArgument(
        'gpp_bim', default_value='True', description='Whether to launch GPP-BIM'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'navbim_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value='',  # Empty means auto-detect based on ifc
        description='Full path to the RVIZ config file to use. If empty, will search for [ifc].rviz in nav_model/ directory, otherwise use navbim_default_view.rviz',
    )

    declare_bim_alignment_cmd = DeclareLaunchArgument(
        'bim_alignment',
        default_value='static',
        description='BIM alignment method: "static"',
    )

    bringup_cmd_group = GroupAction(
        [
            # Robot setup (robot_state_publisher with URDF)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'robot_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'robot_name': robot_name,
                }.items(),
            ),
            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                namespace=namespace,
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'map_infrastructure_launch.py')
                ),
                launch_arguments={
                    'nav_model': navigation_model_filepath,
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'use_respawn': use_respawn,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'container_name': 'nav2_container',
                    'log_level': log_level,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'bim_alignment_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'alignment_method': bim_alignment,
                    'log_level': log_level,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'gpp_bim_launch.py')
                ),
                condition=IfCondition(gpp_bim),
                launch_arguments={
                    'nav_model': navigation_model_filepath,
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'use_respawn': use_respawn,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'container_name': 'nav2_container',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'bim_server_launch.py')
                ),
                launch_arguments={
                    'ifc_file': ifc_file,
                    'autostart': autostart,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'rviz_launch.py')
                ),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'namespace': namespace,
                    'rviz_config': rviz_config_file,
                    'use_sim_time': use_sim_time,
                    'nav_model': navigation_model_filepath,
                    'params_file': params_file,
                }.items(),
            )
        ]
    )

    def check_nav_model_and_launch(context, *args, **kwargs):
        """Check if navigation model exists and decide whether to generate it"""
        # Get the actual path values from launch configurations
        nav_model_path = context.launch_configurations.get('nav_model', '')
        force_gen = context.launch_configurations.get('force_nav_model_gen', 'false').lower() == 'true'
        ifc_value = context.launch_configurations.get('ifc', 'Toy_example')
        
        # Resolve rviz_config: check if ifc-specific rviz config exists
        rviz_config_arg = context.launch_configurations.get('rviz_config', '')
        if not rviz_config_arg:  # Empty means auto-detect
            ifc_specific_rviz = os.path.join(bringup_dir, 'nav_model', f'{ifc_value}/{ifc_value}.rviz')
            
            if os.path.exists(ifc_specific_rviz):
                resolved_rviz_config = ifc_specific_rviz
                print(f'Using IFC-specific RViz config: {resolved_rviz_config}')
            else:
                resolved_rviz_config = os.path.join(bringup_dir, 'rviz', 'navbim_default_view.rviz')
                print(f'Using default RViz config: {resolved_rviz_config}')
            
            # Update the context with resolved rviz config
            context.launch_configurations['rviz_config'] = resolved_rviz_config
        
        actions = []
        
        # Check if nav model directory exists
        nav_model_exists = os.path.exists(nav_model_path) and os.path.isdir(nav_model_path)
        
        if force_gen or not nav_model_exists:
            if not nav_model_exists:
                print(f'Navigation model folder not found: {nav_model_path}')
                print('The navigation model will be generated before starting the navigation stack.')
            else:
                print('Force regeneration of navigation model enabled.')
            
            # Use ExecuteProcess to run nav model generation as a subprocess
            cmd_args = [
                'ros2', 'launch', 
                os.path.join(launch_dir, 'nav_model_gen_launch.py'),
                f'ifc_file:={context.launch_configurations.get("ifc_file", "")}',
                f'nav_model:={nav_model_path}',
                f'params_file:={context.launch_configurations.get("params_file", "")}',
                f'autostart:={context.launch_configurations.get("autostart", "true")}',
                f'log_level:={context.launch_configurations.get("log_level", "info")}',
            ]
            
            # Only add namespace if it's not empty
            namespace_val = context.launch_configurations.get('namespace', '')
            if namespace_val:
                cmd_args.append(f'namespace:={namespace_val}')
            
            nav_model_gen_cmd = ExecuteProcess(
                cmd=cmd_args,
                output='screen'
            )
            
            # Event handler to start bringup after nav model generation completes
            event_handler = RegisterEventHandler(
                OnProcessExit(
                    target_action=nav_model_gen_cmd,
                    on_exit=[bringup_cmd_group]
                )
            )
            
            actions.append(nav_model_gen_cmd)
            actions.append(event_handler)
        else:
            print(f'Using existing navigation model at: {nav_model_path}')
            # If nav model exists, launch bringup directly
            actions.append(bringup_cmd_group)
        
        return actions

    # Dynamic launch function
    dynamic_launch = OpaqueFunction(function=check_nav_model_and_launch)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_ifc_cmd)
    ld.add_action(declare_ifc_file_cmd)
    ld.add_action(declare_navigation_model_cmd)
    ld.add_action(declare_force_nav_model_gen_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_gpp_bim_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_bim_alignment_cmd)

    # Add the actions to launch all of the navigation nodes
    # The dynamic_launch function handles both nav model generation (if needed) and bringup
    ld.add_action(dynamic_launch)

    return ld