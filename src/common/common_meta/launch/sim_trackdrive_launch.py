import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        create_node(pkg='path_planning',
                    params=[{'v_max': 7.0,
                             'ax_max': 5.0,
                             'ay_max': 5.0}]),
        create_node(pkg='controller',
                    params=[{'min_cmd': -10.0,
                             'max_cmd': 10.0,
                             'use_optimized_trajectory': True}]),
        create_node(pkg='visualization'),
        create_node(pkg='arussim_interface'),
        create_node(pkg='car_state', 
                    params=[{'simulation': True, 
                    'mission': 'trackdrive',
                    'trackdrive_laps': 10,
                    'extensometer_topic': '/arussim/extensometer',
                    'imu_ax_topic': '/arussim/IMU/ax',
                    'imu_ay_topic': '/arussim/IMU/ay',
                    'imu_r_topic': '/arussim/IMU/yaw_rate',
                    'fr_wheel_speed_topic': '/arussim/fr_wheel_speed',
                    'fl_wheel_speed_topic': '/arussim/fl_wheel_speed',
                    'rr_wheel_speed_topic': '/arussim/rr_wheel_speed',
                    'rl_wheel_speed_topic': '/arussim/rl_wheel_speed'}]),
        create_node(pkg='graph_slam',
                    params=[{'perception_topic': '/arussim/perception',
                             'verbose': False}]),
        create_node(pkg='trajectory_optimization',
                    params=[{'v_max': 10.0,
                             'd_min': 1.3,
                             'mu_y': 1.0,
                             'mu_throttle': 0.6,
                             'mu_brake': 0.6}])
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