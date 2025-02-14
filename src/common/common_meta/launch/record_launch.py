import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from datetime import datetime
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    home_dir = os.path.expanduser("~")
    output_dir = os.path.join(home_dir, f"ARUS_logs/bagfiles/trackdrive_bag_{timestamp}")

    # Record rosbag with mcap extension in steps of 10MB
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', 
             '-o', output_dir, '-s', 'mcap', '--max-bag-size', '10000000'],
        output='screen'
    )

    rslidar_launch = os.path.join(get_package_share_directory('rslidar_sdk'), 
                                        'launch', 'start.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rslidar_launch)),
        create_node(pkg='can_interface'),
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
