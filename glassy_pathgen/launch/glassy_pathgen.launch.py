#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Get the name of the .yaml configuration file either from the package or an external source
    glassy_pathgen_config_arg = DeclareLaunchArgument(
        'glassy_pathgen_yaml', 
        default_value=os.path.join(get_package_share_directory('glassy_pathgen'), 'config', 'glassy_pathgen_params.yaml'),
        description='The configurations for the path gen node')

    glassy_pathgen_node = Node(
                package='glassy_pathgen',
                namespace=[],
                executable='glassy_pathgen',
                parameters=[ 
                    # Pass the file which contains the topics configuration and rates for telemetry
                    LaunchConfiguration('glassy_pathgen_yaml'),
                ],
                output='screen'
            )
    print('Finshing generating the launch description')


    return LaunchDescription([
        glassy_pathgen_config_arg,
        glassy_pathgen_node    
        ])