#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Get the name of the .yaml configuration file either from the package or an external source
    glassy_planning_config_arg = DeclareLaunchArgument(
        'glassy_planning_yaml', 
        default_value=os.path.join(get_package_share_directory('glassy_planning'), 'config', 'path_manager_config.yaml'),
        description='The configurations for the path gen node')

    glassy_planning_node = Node(
                package='glassy_planning',
                namespace=[],
                executable='glassy_planning',
                parameters=[ 
                    # Pass the file which contains the topics configuration and rates for telemetry
                    LaunchConfiguration('glassy_planning_yaml'),
                ],
                output='screen'
            )
    print('Finshing generating the launch description')


    return LaunchDescription([
        glassy_planning_config_arg,
        glassy_planning_node    
        ])