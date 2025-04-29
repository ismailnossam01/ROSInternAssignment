from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, EnvironmentVariable

def generate_launch_description():
    xacro_exe = FindExecutable(name="xacro")

    urdf_file = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'ros2_ws', 'src', 'my_robot_sim', 'urdf', 'limo_ur5e.xacro'
    ])

    srdf_file = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'ros2_ws', 'src', 'my_robot_sim', 'moveit_config', 'limo_ur5e.srdf'
    ])

    moveit_config_file = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'ros2_ws', 'src', 'my_robot_sim', 'moveit_config', 'moveit_params.yaml'
    ])

    robot_description = Command([xacro_exe, ' ', urdf_file])

    robot_description_semantic = Command(['cat ', srdf_file])

    return LaunchDescription([
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                moveit_config_file
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'ros2_ws', 'src', 'my_robot_sim', 'moveit_config', 'moveit.rviz'
            ])],
            output='screen'
        )
    ])
