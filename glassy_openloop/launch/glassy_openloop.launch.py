#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Get the name of the .yaml configuration file either from the package or an external source
    glassy_openloop_config_arg = DeclareLaunchArgument(
        'glassy_openloop_yaml', 
        default_value=os.path.join(get_package_share_directory('glassy_openloop'), 'config', 'open_loop_params.yaml'),
        description='The configurations for the open loop values')

    print(os.path.join(get_package_share_directory('glassy_openloop'), 'config', 'open_loop_params.yaml'))
    glassy_openloop_node = Node(
                package='glassy_openloop',
                namespace=[],
                executable='glassy_openloop',
                parameters=[ 
                    # Pass the file which contains the topics configuration and rates for telemetry
                    LaunchConfiguration('glassy_openloop_yaml'),
                ]
            )



    return LaunchDescription([
        glassy_openloop_config_arg,
        glassy_openloop_node    
        ])