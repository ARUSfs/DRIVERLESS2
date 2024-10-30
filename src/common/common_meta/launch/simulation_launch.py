import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    planning_launch_path = os.path.join(get_package_share_directory('path_planning'), 
                                        'launch', 'path_planning_launch.py')
    controller_launch_path = os.path.join(get_package_share_directory('controller'),
                                        'launch', 'controller_launch.py')
    car_state_launch_path = os.path.join(get_package_share_directory('car_state'),
                                        'launch', 'car_state_launch.py')
    visualization_launch_path = os.path.join(get_package_share_directory('visualization'), 
                                        'launch', 'visualization_launch.py')
    arussim_interface_launch_path = os.path.join(get_package_share_directory('arussim_interface'), 
                                        'launch', 'arussim_interface_launch.py')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(car_state_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arussim_interface_launch_path)),
    ])