#!/usr/bin/env python3
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='glassy_pathplanning',
            namespace='glassy',
            executable='glassy_pathplanning',
            name='glassy_pathplanning',
            output = 'screen'
        ),
    ])