import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    can_interface_launch_path = os.path.join(get_package_share_directory('can_interface'), 
                                        'launch', 'can_interface_launch.py')
    epos_interface_launch_path = os.path.join(get_package_share_directory('epos_interface'), 
                                        'launch', 'epos_interface_launch.py')
    inspection_control_launch_path = os.path.join(get_package_share_directory('inspection_control'),
                                        'launch', 'inspection_control_launch.py')
    car_state_launch_path = os.path.join(get_package_share_directory('car_state'),
                                        'launch', 'car_state_launch.py')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(can_interface_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(epos_interface_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(inspection_control_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(car_state_launch_path))
    ])