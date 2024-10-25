import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_state', 
            executable='car_state',  
            name='car_state', 
            output='screen', 
        )
    ])
