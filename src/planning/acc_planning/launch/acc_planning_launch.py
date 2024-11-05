import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "acc_planning"



    return LaunchDescription([
        Node(
            package="acc_planning",
            executable="acc_planningexec",
            name="acc_planning,
            output="screen"
            
        )
    ])