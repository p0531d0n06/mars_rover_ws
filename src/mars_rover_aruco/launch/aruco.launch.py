"""Launch the ArUco detector and waypoint bridge nodes."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mars_rover_aruco')
    cfg = os.path.join(pkg, 'config', 'aruco_waypoints.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    aruco_detector = Node(
        package='mars_rover_aruco',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[cfg, {'use_sim_time': use_sim_time}],
    )

    waypoint_bridge = Node(
        package='mars_rover_aruco',
        executable='aruco_waypoint_bridge',
        name='aruco_waypoint_bridge',
        output='screen',
        parameters=[cfg, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        aruco_detector,
        waypoint_bridge,
    ])
