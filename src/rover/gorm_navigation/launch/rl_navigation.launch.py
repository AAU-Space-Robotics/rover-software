from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for GORM RL-based navigation.
    
    This launch file starts the RL navigation node with configurable parameters.
    """
    
    # Declare launch arguments
    triton_server_url_arg = DeclareLaunchArgument(
        'triton_server_url',
        default_value='localhost:8000',
        description='URL of the Triton Inference Server'
    )
    
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='IQL_depth',
        description='Name of the RL model in Triton'
    )
    
    model_version_arg = DeclareLaunchArgument(
        'model_version',
        default_value='1',
        description='Version of the RL model'
    )
    
    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='5.0',
        description='Inference rate in Hz'
    )
    
    enable_triton_arg = DeclareLaunchArgument(
        'enable_triton',
        default_value='true',
        description='Enable Triton inference (set to false for testing without Triton)'
    )
    
    # RL Navigation Node
    rl_navigation_node = Node(
        package='gorm_navigation',
        executable='rl_navigation_node',
        name='gorm_rl_navigation',
        output='screen',
        parameters=[{
            'triton_server_url': LaunchConfiguration('triton_server_url'),
            'model_name': LaunchConfiguration('model_name'),
            'model_version': LaunchConfiguration('model_version'),
            'inference_rate': LaunchConfiguration('inference_rate'),
            'depth_image_height': 90,
            'depth_image_width': 160,
            'max_depth_value': 4.0,
            'enable_triton': LaunchConfiguration('enable_triton'),
        }]
    )
    
    return LaunchDescription([
        triton_server_url_arg,
        model_name_arg,
        model_version_arg,
        inference_rate_arg,
        enable_triton_arg,
        rl_navigation_node,
    ])
