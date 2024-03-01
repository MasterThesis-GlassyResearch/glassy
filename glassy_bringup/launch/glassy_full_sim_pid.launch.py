import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   
   # TODO - add some adaptability, maybe change so model launched is defined here
   # launch the simulation
   ocean_world = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_simulation'), 'launch'),
         '/ocean_world.launch.py'])
      )
   glassy_vehicle = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_simulation'), 'launch'),
         '/glassy_model.launch.py'])
      )
   
    # start main node
   glassy_main = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_main'), 'launch'),
         '/glassy_main.launch.py'])
      )

    # start joystick nodes

   glassy_joystick = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_joystick'), 'launch'),
         '/glassy_joystick_full.launch.py'])
      )
   
    # start inner loop nodes

   glassy_inner = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('glassy_innerloop'), 'launch'),
         '/glassy_innerloop_full.launch.py'])
      )

   return LaunchDescription([
      ocean_world,
      glassy_vehicle,
      glassy_main,
      glassy_joystick,
      glassy_inner
   ])