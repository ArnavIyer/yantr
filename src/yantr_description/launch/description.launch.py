from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = PathJoinSubstitution([
        FindPackageShare('yantr_description'),
        'urdf',
        'yantr.urdf.xacro',
    ])

    robot_description = {
        'robot_description': Command(['xacro ', description_file])
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
    ])
