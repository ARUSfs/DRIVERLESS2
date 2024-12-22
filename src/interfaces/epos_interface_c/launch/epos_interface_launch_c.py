from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = "epos_interface_c"
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, "config", "epos_interface_config_c.yaml")
    return LaunchDescription([
        Node(
            package=package_name,
            executable='epos_interface_exec',
            name='epos_interface_c',
            output='screen',
            parameters=[config_file]
        )
    ])