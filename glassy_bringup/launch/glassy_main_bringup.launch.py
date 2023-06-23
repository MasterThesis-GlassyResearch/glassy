#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glassy_main',
            namespace='glassy',
            executable='glassy_main',
            name='glassy_main',
            output='screen'
        ),
        Node(
            package='glassy_joystick',
            namespace='glassy',
            executable='glassy_joystick',
            name='glassy_joystick'
        ),
        Node(
            package='joy',
            namespace='glassy',
            executable='joy_node',
            name='joystick_node'
        ),
    ])