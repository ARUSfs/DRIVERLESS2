import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        create_node(pkg='controller',
                    params=[{'trajectory': '/arussim_interface/fixed_trajectory'}]),
        create_node(pkg='visualization'),
        create_node(pkg='arussim_interface'),
        create_node(pkg='car_state', 
                    params=[{'simulation': True, 
                    'mission': 'trackdrive',          
                    'get_arussim_ground_truth': True}])
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