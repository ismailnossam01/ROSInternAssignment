from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import EnvironmentVariable, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xacro_exe = FindExecutable(name="xacro")

    urdf_file = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'ros2_ws', 'src', 'my_robot_sim', 'urdf', 'limo_ur5e.xacro'
    ])

    robot_description = Command([xacro_exe, ' ', urdf_file])

    return LaunchDescription([
        # Start Gazebo Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gzserver.launch.py'
                )
            ),
            launch_arguments={'world': PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'ros2_ws', 'src', 'my_robot_sim', 'worlds', 'my_world.world'
            ])}.items(),
        ),
        # Start Gazebo Client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gzclient.launch.py'
                )
            )
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Spawn Robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'limo_ur5e'],
            output='screen'
        ),
    ])
