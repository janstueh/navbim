#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
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

    # Lifecycle nodes for non-composition mode
    # Note: Map infrastructure nodes (topomap_server, room_tracker, multimap_server)
    #       are managed by lifecycle_manager_map_infrastructure in map_infrastructure_launch.py
    # Note: global_costmap is a sub-node of room_planner_server, so not listed separately
    # Note: clearance_costmap is a sub-node of clearance_server, so not listed separately
    # Note: bim_server is Python and launched separately (not managed by lifecycle_manager)
    lifecycle_nodes = [
        'gpp_bim',             # Global path planner
        'room_planner_server', # Room-level planning (manages global_costmap as sub-node)
        'clearance_server',    # Clearance calculation (manages clearance_costmap as sub-node)
        'bt_navigator',        # Behavior tree navigator (includes inspection planning)
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
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

    # Create a Python expression to extract the navigation model name from the path
    # This handles both default paths and custom paths
    navigation_model_name = PythonExpression([
        "'", navigation_model_filepath, "'.split('/')[-1]"
    ])

    # Standalone nodes (non-composition mode)
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(namespace=namespace),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_gpp_bim',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'autostart': autostart, 
                    'node_names': lifecycle_nodes,
                    'bond_timeout': 30.0,  # Increased timeout for Python nodes
                    'bond_respawn_max_duration': 30.0,
                    'service_timeout': 15.0,  # Increased for bt_navigator behavior tree loading
                }],
            ),
            Node(
                package='navbim_gpp_bim',
                executable='gpp_bim_server',
                name='gpp_bim',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params,
                    {
                        'nav_model': navigation_model_filepath,
                        'topomap_file': [navigation_model_filepath, '/', navigation_model_name, '.json']
                    }],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='navbim_gpp_bim',
                executable='room_planner_server',
                name='room_planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='navbim_gpp_bim',
                executable='clearance_server',
                name='clearance_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='navbim_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
        ],
    )

    # Composition mode
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(namespace=namespace),
            
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='navbim_gpp_bim',
                        plugin='navbim_gpp_bim::GppBimServer',
                        name='gpp_bim',
                        parameters=[configured_params,
                            {
                                'nav_model': navigation_model_filepath,
                                'topomap_file': [navigation_model_filepath, '/', navigation_model_name, '.json']
                            }],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='navbim_gpp_bim',
                        plugin='navbim_gpp_bim::RoomPlannerServer',
                        name='room_planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='navbim_gpp_bim',
                        plugin='navbim_gpp_bim::ClearanceServer',
                        name='clearance_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='navbim_bt_navigator',
                        plugin='navbim_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_gpp_bim',
                        parameters=[{
                            'autostart': autostart,
                            'node_names': lifecycle_nodes,
                            'bond_timeout': 10.0,
                            'bond_respawn_max_duration': 10.0,
                        }],
                    ),
                ],
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
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
