from launch import LaunchDescription
from launch_ros.actions import Node


"""
This is safe stop directly control cmd vel and suppose it has someone else ouput 
in cmd_vel topic for robot velocity
"""


def generate_launch_description():
    return LaunchDescription([
        
        # launch safe stop node
        Node(
            package="my_mcl_pkg",
            executable="safe_stop_cmd",
            name="safe_stop_cmdvel_node",
            output="screen",
        )
    ])
