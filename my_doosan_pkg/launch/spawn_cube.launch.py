import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the URDF file
    cube_urdf = os.path.join(get_package_share_directory('my_doosan_pkg'),'description', 'urdf', 'cube.urdf')

    # Spawn the cube entity in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-entity', 'cube', '-file', cube_urdf, '-x', '0.5', '-y', '0.6', '-z', '0.5'],  # Increased z to 0.2
                    output='screen')


    return LaunchDescription([
        spawn_entity,
    ])
