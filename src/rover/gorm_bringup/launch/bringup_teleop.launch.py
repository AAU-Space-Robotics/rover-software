
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

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

    # Include control switching (twist_mux) system
    control_switch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gorm_bringup'), 'launch'),
            '/control_switch.launch.py'
        ])
    )

    # Include teleop system  
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gorm_teleop'), 'launch'),
            '/teleop.launch.py'
        ])
    )

    # Web interface (includes rosbridge and web server)
    web_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gorm_web_interface'), 'launch'),
            '/web_interface.launch.py'
        ])
    )

    ld.add_action(control_switch)
    ld.add_action(teleop_launch) 
    ld.add_action(ackermann_node)
    ld.add_action(motor_driver_node)
    ld.add_action(web_interface)

    return ld
