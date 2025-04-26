import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "icp_slam"
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, "config", package_name+"_config.yaml")

    return LaunchDescription([
        Node(
            package=package_name,
            executable=package_name+"_exec",
            name=package_name,
            output="screen",
            parameters=[config_file]
        )
    ])