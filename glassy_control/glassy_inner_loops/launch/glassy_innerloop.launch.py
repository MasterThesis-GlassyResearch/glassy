#!/usr/bin/env python3
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='glassy_innerloop',
            namespace='glassy',
            executable='glassy_innerloop',
            name='glassy_innerloop',
            output = 'screen'
        ),
    ])