from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'api_url',
            default_value='http://localhost:8000',
            description='URL of the FastAPI vision server'
        ),
        DeclareLaunchArgument(
            'send_distance_m',
            default_value='0.25',
            description='Distance in meters between image uploads'
        ),

        Node(
            package='camera_client',
            executable='camera_client_node',
            name='camera_client_node',
            parameters=[{
                'api_url': LaunchConfiguration('api_url'),
                'send_distance_m': LaunchConfiguration('send_distance_m'),
            }],
            output='screen'
        )
    ])
