import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   
    # start main node
   glassy_main = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_px4_manager'), 'launch'),
         '/glassy_px4_manager.launch.py'])
      )

   
    # start inner loop nodes

   glassy_openloop = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_openloop'), 'launch'),
         '/glassy_openloop.launch.py'])
      )

   return LaunchDescription([
      glassy_main,
      glassy_openloop
   ])