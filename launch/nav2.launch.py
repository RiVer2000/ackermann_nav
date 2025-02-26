from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    ackermann_nav_dir = get_package_share_directory('ackermann_nav')
    map_file = os.path.join(ackermann_nav_dir, 'maps', 'map.yaml')
    params_file = os.path.join(ackermann_nav_dir, 'config', 'nav2_params.yaml')
    urdf_file = os.path.join(ackermann_nav_dir, 'urdf', 'ackermann_robot.xacro')

    # Convert Xacro to URDF
    robot_description = Command(['xacro ', urdf_file])

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

        # Start AMCL for Localization
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            parameters=[{
                "use_sim_time": True
            }],
            output="screen"
        ),

        # Lifecycle Manager (to ensure Map Server activates correctly)
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "amcl"]
            }],
            output="screen"
        ),

        # Static Tf from map to odom (instead of directly to base_link)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen"
        ),

        # Robot State Publisher (Needed for TF and URDF)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': robot_description}],
            output="screen"
        ),

        # Start Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': params_file,
                'map': map_file
            }.items()
        )
    ])

