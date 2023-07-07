#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import sys

    

def generate_launch_description():
    
    
    # initialize the port
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='serial:///dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:57600'
    )

    forwarding_arg = DeclareLaunchArgument(
        'forwarding',
        default_value='True'
    )

    return LaunchDescription([
        Node(
            package='glassy_main',
            namespace='glassy',
            executable='glassy_main',
            name='main',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'forwarding': LaunchConfiguration('forwarding'),
            }],
            output='screen'
        ),
    ])