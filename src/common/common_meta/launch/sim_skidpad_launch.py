import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        create_node(pkg='skidpad_planning',
                    params=[{'target_first_lap': 5.0, 
                    'target_second_lap': 10.0,
                    'top_accx_forwards': 5.0,
                    'top_accx_backwards': 3.0,
                    'top_accy': 6.0,
                    'step_width_1': 0.0,
                    'step_width_2': 0.0,
                    'debug': False}]),
        create_node(pkg='controller',
                    params=[{'trajectory_topic': "/skidpad_planning/trajectory",
                             'min_cmd': -10.0,
                             'max_cmd': 10.0,
                             'look_ahead_distance': 6.0,
                             'debug': False}]),
        create_node(pkg='visualization'),
        create_node(pkg='arussim_interface'),
        create_node(pkg='car_state', 
                    params=[{'simulation': True, 
                    'mission': 'skidpad',
                    'extensometer_topic': '/arussim/extensometer',
                    'imu_ax_topic': '/arussim/IMU/ax',
                    'imu_ay_topic': '/arussim/IMU/ay',
                    'imu_r_topic': '/arussim/IMU/yaw_rate',
                    'fr_wheel_speed_topic': '/arussim/fr_wheel_speed',
                    'fl_wheel_speed_topic': '/arussim/fl_wheel_speed',
                    'rr_wheel_speed_topic': '/arussim/rr_wheel_speed',
                    'rl_wheel_speed_topic': '/arussim/rl_wheel_speed',
                    'debug': False}]),
        create_node(pkg='graph_slam',
                    params=[{'perception_topic': '/arussim/perception',
                             'debug': False}])
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