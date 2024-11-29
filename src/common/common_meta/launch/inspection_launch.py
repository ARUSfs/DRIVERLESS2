import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime


def generate_launch_description():

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_dir = f"/home/arus/.ros/inspection_bag_{timestamp}"

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', 
             '-o', output_dir],
        output='screen'
    )

    return LaunchDescription([
        create_node(pkg='can_interface'),
        create_node(pkg='epos_interface', 
                    exec='steering_handle.py'),
        create_node(pkg='inspection_control'),
        create_node(pkg='car_state', 
                    params=[{'simulation': False}]),
        rosbag_record
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