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
        launch_arguments={'device_name': '/dev/ttyUSB1', 'device_baudrate': '115200'}.items()
    )
    lidar_reader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lidar_reader'), 'launch'),
            '/lidar_launch.py'
        ]),
        launch_arguments={'device_name': '/dev/ttyUSB0', 'device_baudrate': '230400'}.items()
    )

    cartographer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('cartographer_ros'), 'launch'),
            '/my_robot.launch.py'
        ])
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
            #bau_ctrl,
            lidar_reader,
            cartographer_node
        ])

