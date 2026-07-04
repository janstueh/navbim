#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.actions


def generate_launch_description():
    """Generate launch description for BIM server."""
    
    # Declare launch arguments
    ifc_file_arg = DeclareLaunchArgument(
        'ifc_file',
        default_value='',
        description='Path to the IFC file to load'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate the lifecycle node'
    )
    
    # Kill any existing bim_server process to avoid duplicate nodes
    cleanup_old_bim_server = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'if ros2 node list 2>/dev/null | grep -q "/bim_server"; then '
            '  echo "Found existing BIM server, checking current state..."; '
            '  STATE=$(ros2 lifecycle get /bim_server 2>&1 | grep -oP "\\[\\d+\\]" | tr -d "[]"); '
            '  if [ "$STATE" = "3" ]; then '
            '    echo "Deactivating..."; '
            '    ros2 lifecycle set /bim_server deactivate 2>/dev/null && sleep 0.3; '
            '  fi; '
            '  STATE=$(ros2 lifecycle get /bim_server 2>&1 | grep -oP "\\[\\d+\\]" | tr -d "[]"); '
            '  if [ "$STATE" = "2" ]; then '
            '    echo "Cleaning up..."; '
            '    ros2 lifecycle set /bim_server cleanup 2>/dev/null && sleep 0.3; '
            '  fi; '
            '  echo "Shutting down..."; '
            '  if ros2 lifecycle set /bim_server shutdown 2>/dev/null; then '
            '    echo "Shutdown successful"; '
            '  else '
            '    echo "Shutdown failed, force killing..."; '
            '    pkill -f "navbim_bim_server.*bim_server" 2>/dev/null || true; '
            '  fi; '
            '  echo "Cleanup complete"; '
            'else '
            '  echo "No existing BIM server found"; '
            'fi'
        ],
        output='screen'
    )
    
    # Create BIM server node as a regular Node (not LifecycleNode action)
    # This allows the executable to manage its own lifecycle or be managed externally
    # Delay start to ensure cleanup completes first
    bim_server_node = TimerAction(
        period=2.0,  # Wait 2 seconds for cleanup to complete
        actions=[
            Node(
                package='navbim_bim_server',
                executable='bim_server',
                name='bim_server',
                namespace='',
                output='screen',
                parameters=[{
                    'ifc_file': LaunchConfiguration('ifc_file'),
                }]
            )
        ]
    )
    
    # Use a bash script to wait for node and transition states properly
    configure_and_activate = TimerAction(
        period=4.0,  # Wait 4 seconds (2s cleanup + 2s node start)
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'for i in {1..10}; do '
                    '  if ros2 node list | grep -q "/bim_server"; then '
                    '    echo "BIM server node found, checking state..."; '
                    '    STATE=$(ros2 lifecycle get /bim_server 2>&1); '
                    '    echo "Current state: $STATE"; '
                    '    if echo "$STATE" | grep -q "unconfigured \\[1\\]"; then '
                    '      echo "Configuring BIM server..."; '
                    '      if ros2 lifecycle set /bim_server configure; then '
                    '        echo "Configuration successful"; '
                    '        STATE=$(ros2 lifecycle get /bim_server 2>&1); '
                    '        echo "New state: $STATE"; '
                    '      else '
                    '        echo "Configuration failed"; exit 1; '
                    '      fi; '
                    '    fi; '
                    '    if echo "$STATE" | grep -q "inactive \\[2\\]"; then '
                    '      echo "Activating BIM server..."; '
                    '      if ros2 lifecycle set /bim_server activate; then '
                    '        echo "Activation successful"; '
                    '      else '
                    '        echo "Activation failed"; exit 1; '
                    '      fi; '
                    '    elif echo "$STATE" | grep -q "active \\[3\\]"; then '
                    '      echo "BIM server already active"; '
                    '    fi; '
                    '    exit 0; '
                    '  fi; '
                    '  sleep 0.5; '
                    'done; '
                    'echo "ERROR: BIM server node not found"; exit 1'
                ],
                output='screen'
            )
        ],
        condition=IfCondition(LaunchConfiguration('autostart'))
    )
    
    return LaunchDescription([
        ifc_file_arg,
        autostart_arg,
        cleanup_old_bim_server,
        bim_server_node,
        configure_and_activate,
    ])
