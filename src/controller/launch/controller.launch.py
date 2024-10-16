from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the YAML configuration file
    config_file_path = os.path.join(
        get_package_share_directory('controller'),  
        'config',
        'controller_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller',  
            executable='controller_node', 
            name='controller_node',       
            output='screen',
            parameters=[config_file_path] 
        )
    ])
