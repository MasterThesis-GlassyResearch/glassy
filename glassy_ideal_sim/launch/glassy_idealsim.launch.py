#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Get the name of the .yaml configuration file either from the package or an external source
    glassy_idealsim_config_arg = DeclareLaunchArgument(
        'glassy_idealsim_yaml', 
        default_value=os.path.join(get_package_share_directory('glassy_idealsim'), 'config', 'ideal_sim_params.yaml'),
        description='The configurations for the open loop values')

    print(os.path.join(get_package_share_directory('glassy_idealsim'), 'config', 'ideal_sim_params.yaml'))
    glassy_idealsim_node = Node(
                package='glassy_idealsim',
                namespace=[],
                executable='glassy_idealsim',
                parameters=[ 
                    # Pass the file which contains the topics configuration and rates for telemetry
                    LaunchConfiguration('glassy_idealsim_yaml'),
                ]
            )



    return LaunchDescription([
        glassy_idealsim_config_arg,
        glassy_idealsim_node    
        ])