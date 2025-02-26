from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ackermann_nav')
    world_file = os.path.join(pkg_share, 'worlds', 'ackermann_world.world')
    robot_xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_robot.xacro')

    # Convert Xacro to URDF
    robot_description = Command(['xacro ', robot_xacro_file])

    return LaunchDescription([
        # Launch Gazebo with a world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn the Ackermann Robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'ackermann_bot',
                '-topic', '/robot_description'
            ],
            output='screen'
        ),

        # Robot State Publisher (Publishes the URDF from Xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )
    ])

