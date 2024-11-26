import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_mcl = get_package_share_directory('my_mcl_pkg')

    return LaunchDescription([ 
        Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': os.path.join(pkg_mcl, 'config', 'bcr_map.yaml')} ,
            {'use_sim_time': True}        
            ],
        ),
    
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},  # Automatically transition nodes to active state
                {'node_names': ['map_server']}  # Only manage map_server
            ]
        )


    ])
