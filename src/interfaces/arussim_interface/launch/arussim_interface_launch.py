import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "arussim_interface"
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, "config", "arussim_interface_config.yaml")

    return LaunchDescription([
        Node(
            package=package_name,
            executable="arussim_interface_exec",
            name="arussim_interface",
            output="screen",
            parameters=[config_file]
        )
    ])