#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for twist_mux multiplexer to switch between joystick, remote, and autonomous control.
    
    This launch file sets up the twist_mux node with proper configuration and remappings
    to handle multiple velocity command sources with priority-based switching.
    """
    
    # Get the config file path
    twist_mux_config = os.path.join(
        get_package_share_directory('gorm_bringup'),
        'config',
        'twist_mux.yaml'
    )
    
    # twist_mux node - central multiplexer for velocity commands
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel')  # Output directly to what base control expects
        ],
        output='screen'
    )
    
    return LaunchDescription([
        twist_mux_node,
    ])
