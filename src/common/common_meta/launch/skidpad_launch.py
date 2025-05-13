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
    output_dir = os.path.join(home_dir, f"ARUS_logs/bagfiles/skidpad_bag_{timestamp}")

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
        create_node(pkg='epos_interface'),
        create_node(pkg='perception'),
        create_node(pkg='skidpad_planning',
                    params=[{'target_first_lap': 3.0, 
                    'target_second_lap': 3.0,
                    'top_accx_forwards': 5.0,
                    'top_accx_backwards': 3.0,
                    'top_accy': 6.0,
                    'step_width_1': 0.0,
                    'step_width_2': 0.0}]),
        create_node(pkg='controller',
                    params=[{'trajectory_topic': "/skidpad_planning/trajectory",
                             'min_cmd': -3.0,
                             'max_cmd': 5.0}]),
        create_node(pkg='graph_slam'),
        create_node(pkg='car_state', 
                    params=[{'simulation': False, 
                    'mission': 'skidpad'}]),
        create_node(pkg='visualization'),
        rosbag_record,
        Node(package='common_meta', executable='param_dumper_exec', name='param_dumper'),
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