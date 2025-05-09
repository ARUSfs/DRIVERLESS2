import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from datetime import datetime
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rviz_config_file = os.path.join(
        get_package_share_directory('fast_limo'),
        'config',
        'rviz',
        'limo.rviz'
    )


    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        create_node(pkg='fast_limo',
            exec='fast_limo_multi_exec',
            config='params.yaml'),
        create_node(pkg='perception_acc'),
        create_node(pkg='graph_slam',
                    params=[{'perception_topic': '/perception_acc/map',
                             'map_topic': '/slam/map_test',
                             'final_map_topic': '/slam/final_map_test',}]),
        create_node(pkg='path_planning',
                    params=[{'v_max': 2.5,
                             'ax_max': 3.0,
                             'ay_max': 3.0,
                             'map_topic': '/slam/map_test',
                             'triangulation_topic': '/path_planning/triangulation_test',
                             'trajectory_topic': '/path_planning/trajectory_test'}]),
        create_node(pkg='visualization',
                    params=[{'triangulation_topic': '/path_planning/triangulation_test',
                             'delaunay_trajectory_topic': '/path_planning/trajectory_test',
                             'triangulation_visualization_topic': '/visualization/triangulation_test',
                             'trajectory_visualization_topic': '/visualization/trajectory_test'}])
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
