import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node


"""
Testing code for map 
"""

def generate_launch_description():

    pkg_mcl = get_package_share_directory('my_mcl_pkg')

    return LaunchDescription([

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                '-d',  
                os.path.join(
                    get_package_share_directory('my_mcl_pkg'),
                    'rviz',
                    'final_rviz.rviz'  
                )
            ],
            parameters=[
                {'use_sim_time': True}
            ],
        ),

        Node(
            package='my_mcl_pkg',  
            executable='test_map',  
            name='test_map_node',  
            output='screen',
            parameters=[
            ]
        ),
    ])
