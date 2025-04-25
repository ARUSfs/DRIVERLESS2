import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "camera_perception"
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, "config", "camera_calibration_config.yaml")

    return LaunchDescription([
        Node(
            package=package_name,
            executable="camera_calibration_exec",
            name="camera_calibration",
            output="screen",
            parameters=[config_file]),
        create_node("perception")
    ])

def create_node(pkg, config=None, exec=None, params=[]): 

    if config is None:
        config = pkg + "_config.yaml"
    if exec is None:
        exec = pkg + "_exec"

    package_share_directory = get_package_share_directory(pkg)
    config_file = os.path.join(package_share_directory, "config", config)

    return Node(
        package=pkg,
        executable=exec,
        name=pkg,
        output="screen",
        parameters=[config_file]+params
        )