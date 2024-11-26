from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        # launch safe stop node
        Node(
            package="my_mcl_pkg",
            executable="safe_stop",
            name="safe_stop_node",
            output="screen",
        )
    ])
