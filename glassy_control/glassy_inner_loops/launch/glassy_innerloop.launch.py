#!/usr/bin/env python3
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('glassy_innerloop'),
      'config',
      'params.yaml'
      )
    print(config)


    return LaunchDescription([
        Node(
            package='glassy_innerloop',
            executable='glassy_innerloop',
            namespace='',
            name='glassy_innerloop',
            parameters=[config],
            output='screen'
        )
   ])