# car_state_launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This function generates a launch description for the ROS 2 node
    return LaunchDescription([
        Node(
            package='car_state',  # Name of the package where your node is located
            executable='car_state_node',  # Name of the executable for your node
            name='car_state_node',  # Name of the node (can be changed if necessary)
            output='screen',  # Output of the node to the console
            parameters=[{
                # Here you can add parameters if necessary
            }],
            remappings=[
                # Here you can add topic remappings if necessary
            ]
        )
    ])
