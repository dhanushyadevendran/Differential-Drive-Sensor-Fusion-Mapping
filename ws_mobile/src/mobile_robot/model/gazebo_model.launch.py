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

    path_model_file = os.path.join(get_package_share_directory(package_name), model_file_rel_path)

    robot_description = xacro.process_file(path_model_file).toxml()

    gazebo_launch_file = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )

    gazebo_launch = IncludeLaunchDescription(
    gazebo_launch_file,
        launch_arguments={
            'world': os.path.join(get_package_share_directory(package_name), 'gazebo_worlds', 'apartment.world')
        }.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'robot.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_launch,
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_model_node,
        rviz_node
    ])