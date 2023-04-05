from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    device_name_arg = DeclareLaunchArgument(
        'device_name', default_value=TextSubstitution(text='/dev/ttyUSB0')
    )
    device_baudrate_arg = DeclareLaunchArgument(
        'device_baudrate', default_value=TextSubstitution(text='115200')
    )

    wheel_motor_rpm_arg = DeclareLaunchArgument(
        "wheel_motor_rpm", default_value=TextSubstitution(text='150')
    )
    wheel_motor_ratio_arg = DeclareLaunchArgument(
        "wheel_motor_ratio", default_value=TextSubstitution(text='75')
    )
    wheel_encoder_scrl_arg = DeclareLaunchArgument(
        "wheel_encoder_scrl", default_value=TextSubstitution(text='3300')
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", default_value=TextSubstitution(text='32.5')
    )

    return LaunchDescription([
            device_name_arg,
            device_baudrate_arg,
            wheel_motor_rpm_arg,
            wheel_motor_ratio_arg,
            wheel_encoder_scrl_arg,
            wheel_radius_arg,
            Node(
                package='bau_ctrl',
                executable='bau_ctrl',
                name='bau_test',
                parameters=[{
                    'device_name': LaunchConfiguration('device_name'),
                    'device_baudrate': LaunchConfiguration('device_baudrate'),
                    'wheel_motor_rpm': LaunchConfiguration('wheel_motor_rpm'),
                    'wheel_motor_ratio': LaunchConfiguration('wheel_motor_ratio'),
                    'wheel_encoder_scrl': LaunchConfiguration('wheel_encoder_scrl'),
                    'wheel_radius': LaunchConfiguration('wheel_radius')
                }]
            )
        ])

