#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # RViZ2 settings
  rviz2_config = os.path.join(
      get_package_share_directory('lslidar_driver'),
      'rviz',
      'lslidar.rviz'
  )
  
  rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2_show',
      arguments=['-d',rviz2_config],
      output='screen'
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(rviz2_node)
  
  return ld