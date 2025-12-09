from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'joystick_device',
            default_value='/dev/input/js0',
            description='Path to joystick device'
        ),
        DeclareLaunchArgument(
            'wheel_track',
            default_value='0.3175',
            description='Distance between left and right wheels (m)'
        ),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='1.0',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='2.0',
            description='Maximum angular velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'deadband_threshold',
            default_value='0.05',
            description='Deadband threshold for joystick axes'
        ),
        DeclareLaunchArgument(
            'publish_debug',
            default_value='false',
            description='Whether to publish debug joy messages'
        ),
        DeclareLaunchArgument(
            'safety_timeout_ms',
            default_value='500',
            description='Safety timeout in milliseconds'
        ),

        # Joystick node
        Node(
            package='joystick',
            executable='joystick_node',
            name='joystick_node',
            parameters=[{
                'joystick_device': LaunchConfiguration('joystick_device'),
                'wheel_track': LaunchConfiguration('wheel_track'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'deadband_threshold': LaunchConfiguration('deadband_threshold'),
                'publish_debug': LaunchConfiguration('publish_debug'),
                'safety_timeout_ms': LaunchConfiguration('safety_timeout_ms'),
            }],
            output='screen'
        )
    ])