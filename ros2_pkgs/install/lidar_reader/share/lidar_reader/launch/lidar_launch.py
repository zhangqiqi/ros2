from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    device_name_arg = DeclareLaunchArgument(
        'device_name', default_value=TextSubstitution(text='/dev/ttyUSB0')
    )
    device_baudrate_arg = DeclareLaunchArgument(
        'device_baudrate', default_value=TextSubstitution(text='230400')
    )

    return LaunchDescription([
            device_name_arg,
            device_baudrate_arg,
            Node(
                package='lidar_reader',
                executable='lidar_reader',
                name='N10_lidar',
                parameters=[{
                    'device_name': LaunchConfiguration('device_name'),
                    'device_baudrate': LaunchConfiguration('device_baudrate')
                 }],
                remappings = [
                    ('LaserScan', 'scan')
                ]
            )
        ])
