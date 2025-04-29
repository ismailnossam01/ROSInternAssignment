from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, EnvironmentVariable

def generate_launch_description():
    xacro_exe = FindExecutable(name="xacro")

    urdf_file = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'ros2_ws', 'src', 'my_robot_sim', 'urdf', 'limo_ur5e.xacro'
    ])

    controller_config = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'ros2_ws', 'src', 'my_robot_sim', 'config', 'diff_drive_controller.yaml'
    ])

    robot_description = Command([xacro_exe, ' ', urdf_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controller_config],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
