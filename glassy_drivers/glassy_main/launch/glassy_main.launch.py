#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

config = [os.path.join(
    get_package_share_directory('glassy_main'),
    'config',
    'params.yaml'
    )]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glassy_main',
            namespace='glassy',
            executable='glassy_main',
            name='main',
            output='screen',
            parameters = config
        ),
    ])