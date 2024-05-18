#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Get the PX4 location 
    package = 'glassy_simulation'
    simulation_dir = get_package_share_directory(package)

    start_px4_glassy = ExecuteProcess(
        cmd=[[
            'ros2 run '+ package,
            ' '+'start_glassy.bash'
        ]],
        shell=True
    )

    start_dds = ExecuteProcess(
    cmd=[[
        'ros2 run '+ package,
        ' '+'start_dds.bash'
    ]],
    shell=True
    )
    return LaunchDescription([
        start_px4_glassy,
        start_dds
    ])