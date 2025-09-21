from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot')

    world_file = os.path.join(pkg_share, 'gazebo_worlds', 'apartment.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'diff_robot.urdf.xacro')

    return LaunchDescription([
        # World file argument
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Full path to world file to load'
        ),

        # Start Gazebo with ROS plugins
        ExecuteProcess(
            cmd=['gazebo', '--verbose', LaunchConfiguration('world'),
                 '-s', 'libgazebo_ros_factory.so',
                 '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'diff_drive_robot',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )
    ])
