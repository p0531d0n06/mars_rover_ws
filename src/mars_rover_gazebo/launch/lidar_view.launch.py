"""
Standalone RViz launch: rover model + 3D LiDAR point cloud view.
Run this alongside the main simulation to get a focused lidar visualisation.

Usage:
  ros2 launch mars_rover_gazebo lidar_view.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('mars_rover_description')
    pkg_gazebo      = get_package_share_directory('mars_rover_gazebo')

    xacro_file = os.path.join(pkg_description, 'urdf', 'mars_rover.urdf.xacro')
    rviz_config = os.path.join(pkg_gazebo, 'config', 'rover_lidar_view.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot state publisher — needed so RViz can resolve TF and RobotModel
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_lidar_view',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    rviz_lidar = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_lidar_view',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_pub,
        rviz_lidar,
    ])
