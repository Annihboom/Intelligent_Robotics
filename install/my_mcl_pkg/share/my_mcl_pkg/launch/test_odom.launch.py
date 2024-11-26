import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_mcl_pkg',
            executable='test_odom',  
            name='test_odom_node',  
            output='screen',
            parameters=[
            ]
        )
    ])
