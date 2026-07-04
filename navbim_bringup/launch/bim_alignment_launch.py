#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory('navbim_bringup')
    namespace = LaunchConfiguration('namespace')
    ifc = LaunchConfiguration('ifc')
    navigation_model_filepath = LaunchConfiguration('nav_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    alignment_method = LaunchConfiguration('alignment_method')
    
    # Static transform parameters (for alignment_method='static')
    static_x = LaunchConfiguration('static_x')
    static_y = LaunchConfiguration('static_y')
    static_z = LaunchConfiguration('static_z')
    static_yaw = LaunchConfiguration('static_yaw')
    static_pitch = LaunchConfiguration('static_pitch')
    static_roll = LaunchConfiguration('static_roll')

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
        description='IFC model name (used to construct default path for nav_model)'
    )

    declare_navigation_model_cmd = DeclareLaunchArgument(
        'nav_model', 
        default_value=[bringup_dir, '/nav_model/', ifc], 
        description='Full path to the navigation model folder (overrides ifc argument if specified)'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'navbim_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_alignment_method_cmd = DeclareLaunchArgument(
        'alignment_method',
        default_value='static',
        description='BIM alignment method: "static"',
    )
    
    declare_static_x_cmd = DeclareLaunchArgument(
        'static_x',
        default_value='0.0',
        description='Static transform X translation (meters)'
    )
    
    declare_static_y_cmd = DeclareLaunchArgument(
        'static_y',
        default_value='0.0',
        description='Static transform Y translation (meters)'
    )
    
    declare_static_z_cmd = DeclareLaunchArgument(
        'static_z',
        default_value='0.0',
        description='Static transform Z translation (meters)'
    )
    
    declare_static_yaw_cmd = DeclareLaunchArgument(
        'static_yaw',
        default_value='0.0',
        description='Static transform yaw rotation (radians)'
    )
    
    declare_static_pitch_cmd = DeclareLaunchArgument(
        'static_pitch',
        default_value='0.0',
        description='Static transform pitch rotation (radians)'
    )
    
    declare_static_roll_cmd = DeclareLaunchArgument(
        'static_roll',
        default_value='0.0',
        description='Static transform roll rotation (radians)'
    )

    # Static identity transform (ifc = map)
    static_alignment = GroupAction(
        condition=IfCondition(PythonExpression(["'", alignment_method, "' == 'static'"])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(namespace=namespace),
            
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='ifc_to_map_link',
                arguments=[static_x, static_y, static_z, static_yaw, static_pitch, static_roll, 'ifc', 'map'],
                output='screen',
            ),
        ],
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_ifc_cmd)
    ld.add_action(declare_navigation_model_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_alignment_method_cmd)
    ld.add_action(declare_static_x_cmd)
    ld.add_action(declare_static_y_cmd)
    ld.add_action(declare_static_z_cmd)
    ld.add_action(declare_static_yaw_cmd)
    ld.add_action(declare_static_pitch_cmd)
    ld.add_action(declare_static_roll_cmd)
    
    # Add alignment method groups
    ld.add_action(static_alignment)

    return ld
