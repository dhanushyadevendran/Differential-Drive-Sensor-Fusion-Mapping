import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'mobile_robot'
    robot_name = 'differential_drive_robot'
    model_file_rel_path = 'model/robot.xacro'

    # Path to Xacro
    path_model_file = os.path.join(get_package_share_directory(package_name), model_file_rel_path)

    # Convert Xacro → URDF
    robot_description = xacro.process_file(path_model_file).toxml()

    # Gazebo Classic launch
    gazebo_launch_file = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )

    gazebo_launch = IncludeLaunchDescription(
    gazebo_launch_file,
        launch_arguments={
            'world': os.path.join(get_package_share_directory(package_name), 'gazebo_worlds', 'apartment.world')
        }.items()
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn robot
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '2.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        node_robot_state_publisher,
        spawn_model_node
    ])