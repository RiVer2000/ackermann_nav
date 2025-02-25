from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    ackermann_nav_dir = get_package_share_directory('ackermann_nav')
    map_file = os.path.join(ackermann_nav_dir, 'maps', 'map.yaml')

    return LaunchDescription([
        # Start the Map Server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[{
                "use_sim_time": True,
                "yaml_filename": map_file
            }],
            output="screen"
        ),
        
        # Static Tf from map to base_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
            output="screen"
        ),

        # Start Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(ackermann_nav_dir, 'config', 'nav2_params.yaml'),
                'map': map_file
            }.items()
        )
    ])
