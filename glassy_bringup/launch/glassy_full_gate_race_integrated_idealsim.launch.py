import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   
    # start main node
   glassy_idealsim = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_idealsim'), 'launch'),
         '/glassy_idealsim.launch.py'])
      )

   
    
   # in this case, it is integrated
   glassy_outer = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_outerloop'), 'launch'),
         '/glassy_outerloop.launch.py'])
      )

   glassy_pathman = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_pathplanning'), 'launch'),
         '/glassy_pathplanning.launch.py'])
      )
   
   glassy_pathgen = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_pathgen'), 'launch'),
         '/glassy_pathgen_dubins.launch.py'])
      )

   return LaunchDescription([
      glassy_idealsim,
      glassy_outer,
      glassy_pathman,
      glassy_pathgen
   ])