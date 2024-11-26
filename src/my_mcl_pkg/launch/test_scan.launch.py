import os
from launch import LaunchDescription
from launch_ros.actions import Node

"""
Testing code for laser scan
"""

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_mcl_pkg',  
            executable='test_scan',  
            name='test_scan_node',  
            output='screen',
            parameters=[
            ]
        )
    ])
