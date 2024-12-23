from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = "epos_interface"
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, "config", "epos_interface_config.yaml")
    return LaunchDescription([
        Node(
            package=package_name,
            executable='steering_handle.py',
            name='epos_interface',
            output='screen',
            parameters=[config_file]
        )
    ])