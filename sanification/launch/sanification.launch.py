from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():
   
   Sanification = Node(
      package='sanification',
      executable='sanification',
      name='sanification_node',
      output='screen'
   )

   # Sanification_Check = Node(
   #    package='sanification',
   #    executable='sanification_check',
   #    name='sanification_check_node',
   #    output='screen'
   # )

   Energy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('energy_manager'),
                        'launch/energy.launch.py')
        ),
    )

   # Creation of the LaunchDescription
   ld = LaunchDescription()

   ld.add_action(Sanification)
   # ld.add_action(Sanification_Check)
   ld.add_action(Energy_launch)


   return ld