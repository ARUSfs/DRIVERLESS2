import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "can_interface"
    package_share_directory = get_package_share_directory(package_name)

    return LaunchDescription([
        Node(
            package=package_name,
            executable="can_interface_exec",
            name="can_interface",
            output="screen"
        )
    ])