#!/usr/bin/env python3

"""Launch file for map infrastructure nodes shared by both planning and localization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory('navbim_bringup')

    namespace = LaunchConfiguration('namespace')
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
    lifecycle_nodes_all = [
        'topomap_server',
        'room_tracker',
        'multimap_server',
    ]
    
    # Lifecycle nodes for composition mode
    lifecycle_nodes_composition = [
        'topomap_server',
        'room_tracker',
        'multimap_server',
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

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_navigation_model_cmd = DeclareLaunchArgument(
        'nav_model', 
        default_value='', 
        description='Full path to the navigation model folder'
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

    # Create environments_file parameter from nav_model path
    environments_file = [navigation_model_filepath, '/', navigation_model_name, '.yaml']

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(namespace=namespace),
            
            # Topomap nodes (must be first in lifecycle order)
            Node(
                package='navbim_topomap_server',
                executable='topomap_server',
                name='topomap_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {
                        'topomap_file': [navigation_model_filepath, '/', navigation_model_name, '.json'],
                    }
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='navbim_room_tracker',
                executable='room_tracker',
                name='room_tracker',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            
            # Map server
            Node(
                package='navbim_multimap_server',
                executable='multimap_server',
                name='multimap_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params,
                            {'environments_file': environments_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            
            
            # Lifecycle manager for map infrastructure
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_infrastructure',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'autostart': autostart, 
                    'node_names': lifecycle_nodes_all,
                    'bond_timeout': 30.0,
                    'bond_respawn_max_duration': 30.0,
                }],
            ),
            
            # Pose to TF publisher
            Node(
                package='navbim_util',
                executable='pose_to_tf_publisher',
                name='pose_to_tf_publisher',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            )
        ],
    )
    
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(namespace=namespace),
            
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='navbim_topomap_server',
                        plugin='navbim_topomap_server::TopomapServer',
                        name='topomap_server',
                        parameters=[
                            configured_params,
                            {
                                'topomap_file': [navigation_model_filepath, '/', navigation_model_name, '.json'],
                            }
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='navbim_room_tracker',
                        plugin='navbim_room_tracker::RoomTracker',
                        name='room_tracker',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='navbim_multimap_server',
                        plugin='navbim_multimap_server::MultimapServer',
                        name='multimap_server',
                        parameters=[configured_params,
                                    {'environments_file': environments_file}],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_map_infrastructure',
                        parameters=[{
                            'autostart': autostart,
                            'node_names': lifecycle_nodes_composition,
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

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_navigation_model_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    
    # Add the actions to launch all map infrastructure nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
