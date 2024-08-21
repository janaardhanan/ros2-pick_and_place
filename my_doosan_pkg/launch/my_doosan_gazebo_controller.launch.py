import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    robot_model = 'a0912'
    #robot_model = 'm1013'

    xacro_file = os.path.join(get_package_share_directory('my_doosan_pkg'), 'description', 'xacro', f'{robot_model}.urdf.xacro')

    # Command to process the xacro file
    robot_description_command = Command(['xacro ', xacro_file])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': ParameterValue(robot_description_command, value_type=str)}]
    )

    # Spawn the robot in Gazebo
    spawn_entity_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_doosan_robot', '-topic', 'robot_description'],
        output='screen'
    )

    #URDF of cube
    cube_urdf = os.path.join(get_package_share_directory('my_doosan_pkg'),'description', 'urdf', 'cube.urdf')

    # Spawn the cube entity in Gazebo
    spawn_cube = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-entity', 'cube', '-file', cube_urdf, '-x', '1.06', '-y', '0.05', '-z', '0.075'],  # Increased z to 0.2
                    output='screen')

    # Gazebo world
    world_file_name = 'my_empty_world.world'
    world = os.path.join(get_package_share_directory('my_doosan_pkg'), 'worlds', world_file_name)
    gazebo_node = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Load and start the controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_entity_robot,
        spawn_cube,
        gazebo_node,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller
    ])
