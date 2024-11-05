from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "perception"

    return LaunchDescription([

        Node(
            package='perception', 
            executable='perception_exec',  
            name='perception', 
            output='screen'
        )
    ])
