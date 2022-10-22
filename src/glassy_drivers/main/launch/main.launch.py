#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main',
            namespace='main',
            executable='main.cpp',
            name='main'
        ),
    ])