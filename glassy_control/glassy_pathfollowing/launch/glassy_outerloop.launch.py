#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Get the name of the .yaml configuration file either from the package or an external source
    glassy_outerloop_config_arg = DeclareLaunchArgument(
        'glassy_outerloop_yaml', 
        default_value=os.path.join(get_package_share_directory('glassy_outerloop'), 'config', 'glassy_outerloop_params.yaml'),
        description='The configurations for the path gen node')

    glassy_outerloop_node = Node(
                package='glassy_outerloop',
                namespace=[],
                executable='glassy_outerloop',
                parameters=[ 
                    # Pass the file which contains the topics configuration and rates for telemetry
                    LaunchConfiguration('glassy_outerloop_yaml'),
                ],
                output='screen'
                # arguments=['--ros-args', '--log-level', 'DEBUG']
            )
    print('Finshing generating the launch description')


    return LaunchDescription([
        glassy_outerloop_config_arg,
        glassy_outerloop_node    
        ])


# #!/usr/bin/env python3
# import os
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node

# def generate_launch_description():



#     return LaunchDescription([
#         Node(
#             package='glassy_outerloop',
#             namespace='',
#             executable='glassy_outerloop',
#             name='glassy_outerloop',
#             output = 'screen'
#         ),
#     ])
