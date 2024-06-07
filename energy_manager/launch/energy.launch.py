from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
   Visibility = Node(
      package='energy_manager',
      executable='visibility_grid',
   )

   En_calculator = Node(
      package='energy_manager',
      executable='energy_calculator',
   )

   En_plotter = Node(
      package='energy_manager',
      executable='energy_plotter',
      output='screen'
   )           

   # Creation of the LaunchDescription
   ld = LaunchDescription()

   ld.add_action(Visibility)
   ld.add_action(En_calculator)
   ld.add_action(En_plotter)

   return ld