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
        create_node(pkg='epos_interface'),
        create_node(pkg='perception'),
        create_node(pkg='path_planning',
                    params=[{'v_max': 3.0,
                             'ax_max': 3.0,
                             'ay_max': 5.0}]),
        create_node(pkg='controller',
                    params=[{'look_ahead_distance': 4.0,
                             'optimized_look_ahead_distance': 6.5,
                             'min_cmd': -5.0,
                             'max_cmd': 7.0,
                             'use_optimized_trajectory': True,
                             'first_lap_steer_control': "PP",
                             'optimized_steer_control': "MPC"}]),
        create_node(pkg='graph_slam'),
        create_node(pkg='car_state', 
                    params=[{'simulation': False, 
                    'mission': 'trackdrive',
                    'trackdrive_laps': 5,
                    'use_wheelspeeds': True}]),
        create_node(pkg='trajectory_optimization',
                    params=[{'v_max': 15.0,
                             'd_min': 1.3,
                             'mu_y': 1.0,
                             'mu_throttle': 0.5,
                             'mu_brake': 0.35}]),
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
