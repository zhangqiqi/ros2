import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bau_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('bau_ctrl'), 'launch'),
            '/bau_ctrl_launch.py'
        ]),
        launch_arguments={'device_name': '/dev/ttyUSB0', 'device_baudrate': '115200'}.items()
    )
    lidar_reader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lidar_reader'), 'launch'),
            '/lidar_launch.py'
        ]),
        launch_arguments={'device_name': '/dev/ttyUSB1', 'device_baudrate': '230400'}.items()
    )

    cartographer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('cartographer_ros'), 'launch'),
            '/my_robot.launch.py'
        ])
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py'
        ]),
        launch_arguments={'params_file': os.path.join(get_package_share_directory('sys_dispatcher'), 'params', 'nav2_params.yaml'), 'map': os.path.join('tmp', 'map.yaml')}.items() 
    )
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('sys_dispatcher'), 'params', 'robot_localization_ekf.yaml')]
    )

    sys_dispatcher_node = Node(
        package='sys_dispatcher',
        executable='sys_dispatcher',
        name='sys_dispatcher',
        parameters=[{
        }]
    ) 

    return LaunchDescription([
            sys_dispatcher_node, 
            bau_ctrl,
            lidar_reader,
            robot_localization_node,
            cartographer_node,
            nav2_node
        ])

