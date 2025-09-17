
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # cmd_vel to motor commands converter
    ackermann_node = Node(
        package='gorm_base_control',
        executable='ackermann_node',
        name='ackermann_node',
        output='screen'
    )

    motor_driver_node = Node(
        package='gorm_base_control',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        namespace='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Joy to cmd_vel converter (if reimplemented in gorm_teleop)
    joy_to_cmd_vel_node = Node(
        package='gorm_teleop',
        executable='joy_to_cmd_vel_node',
        name='joy_to_cmd_vel_node',
        output='screen'
    )

    ld.add_action(joy_node)
    ld.add_action(joy_to_cmd_vel_node)
    ld.add_action(ackermann_node)
    ld.add_action(motor_driver_node)

    return ld
