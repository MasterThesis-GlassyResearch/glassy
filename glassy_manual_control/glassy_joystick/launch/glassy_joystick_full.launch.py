#!/usr/bin/env python3
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    config = [os.path.join(
        get_package_share_directory('glassy_joystick'),
        'config',
        'params.yaml'
        )]

    return LaunchDescription([
        Node(
            package='glassy_joystick',
            namespace='glassy',
            executable='glassy_joystick',
            name='glassy_joystick',
            parameters = config,
            output = 'screen'
        ),
        Node(
            package='joy',
            namespace='glassy',
            executable='joy_node',
            name='joystick_node'
        ),
    ])