"""
Top-level launch file — starts the complete Mars rover simulation:

  1. Gazebo Harmonic with Mars terrain world
  2. Rover spawn + robot_state_publisher
  3. ROS ↔ Gazebo bridges
  4. robot_localization EKF
  5. slam_toolbox (2D LiDAR mapping)
  6. Nav2 (planning + control)
  7. ArUco detector + waypoint bridge

Usage:
  ros2 launch mars_rover_gazebo simulation.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mars_rover_gazebo')
    pkg_nav    = get_package_share_directory('mars_rover_navigation')
    pkg_aruco  = get_package_share_directory('mars_rover_aruco')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── 1. Gazebo + rover spawn ────────────────────────────────────────
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'spawn_rover.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── 2. Navigation stack (delayed to let Gazebo stabilise) ──────────
    nav_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    # ── 3. ArUco nodes (delayed further — needs camera bridge up) ──────
    aruco_launch = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_aruco, 'launch', 'aruco.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation time'),
        spawn_launch,
        nav_launch,
        aruco_launch,
    ])
