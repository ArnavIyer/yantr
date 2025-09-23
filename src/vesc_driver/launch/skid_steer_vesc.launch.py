from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Left VESC driver
        Node(
            package='vesc_driver',
            executable='single_vesc_driver_node',
            name='left_vesc',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'motor_side': 'left'
            }],
            remappings=[
                ('rpm', 'left_vesc/rpm'),
                ('target_velocity', 'left_vesc/target_velocity')
            ]
        ),
        
        # Right VESC driver
        Node(
            package='vesc_driver',
            executable='single_vesc_driver_node',
            name='right_vesc',
            parameters=[{
                'serial_port': '/dev/ttyACM1',
                'motor_side': 'right'
            }],
            remappings=[
                ('rpm', 'right_vesc/rpm'),
                ('target_velocity', 'right_vesc/target_velocity')
            ]
        ),
        
        # Skid-steer controller
        Node(
            package='vesc_driver',
            executable='skid_steer_controller_node',
            name='skid_steer_controller',
            parameters=[{
                'wheel_track': 0.3175  # dist between wheels on same axle
            }]
        ),
    ])