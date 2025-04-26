import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "trajectory_optimization"
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, "config", "trajectory_optimization_config.yaml")

    return LaunchDescription([
        Node(
            package=package_name,
            executable="trajectory_optimization_exec",
            name="trajectory_optimization",
            output="screen",
            parameters=[config_file]
        )
    ])