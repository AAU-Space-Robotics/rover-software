from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for teleop control, including local and remote joystick processing.
    
    This launch file starts two instances of the joy_to_cmd_vel_node:
    1. For local joystick control, subscribing to /joy/joy and publishing to /joystick/cmd_vel.
    2. For remote joystick control, subscribing to /remote/joy and publishing to /remote/cmd_vel.
    """
    launch_description = LaunchDescription()
    
    # Node for local joystick control
    local_joy_to_cmd_vel_node = Node(
        package='gorm_teleop',
        executable='joy_to_cmd_vel_node',
        name='local_joy_to_cmd_vel_node',
        output='screen',
        parameters=[{
            'joy_topic': '/local/joy',
            'twist_topic': '/joystick/cmd_vel'
        }]
    )

    # Node for remote joystick control
    remote_joy_to_cmd_vel_node = Node(
        package='gorm_teleop',
        executable='joy_to_cmd_vel_node',
        name='remote_joy_to_cmd_vel_node',
        output='screen',
        parameters=[{
            'joy_topic': '/remote/joy',
            'twist_topic': '/remote/cmd_vel'
        }]
    )

    # Local joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        remappings=[
            ('/joy', '/local/joy')
        ]
    )

    # Add actions to launch description
    launch_description.add_action(local_joy_to_cmd_vel_node)
    launch_description.add_action(remote_joy_to_cmd_vel_node)
    launch_description.add_action(joy_node)

    return launch_description
