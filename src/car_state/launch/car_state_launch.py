from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('car_state'),  
        'config',
        'controller_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='car_state', 
            executable='car_state_exec',  
            name='car_state', 
            output='screen',
            parameters=[config_file_path]  
        )
    ])
