from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('fast_limo'),  
        'config',
        'fast_limo_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='fast_limo', 
            executable='fast_limo_multi_exec',  
            name='fast_limo', 
            output='screen',
            parameters=[config_file_path]
        )
    ])
