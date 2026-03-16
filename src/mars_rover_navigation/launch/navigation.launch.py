"""
Launch the full navigation stack:
  - slam_toolbox (online async 2D SLAM)
  - Nav2 (planner + controller + behaviours)

NOTE: EKF (robot_localization) is started in spawn_rover.launch.py so that
odom→base_footprint TF is available from the first sensor message.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('mars_rover_navigation')

    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    def nav2_node(pkg, exe, name=None, **kwargs):
        return Node(
            package=pkg, executable=exe,
            name=name or exe,
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            **kwargs,
        )

    # ── slam_toolbox ──────────────────────────────────────────────────
    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    # ── Nav2 nodes (no docking_server / route_server) ─────────────────
    controller_server   = nav2_node('nav2_controller',   'controller_server')
    smoother_server     = nav2_node('nav2_smoother',     'smoother_server')
    planner_server      = nav2_node('nav2_planner',      'planner_server')
    behavior_server     = nav2_node('nav2_behaviors',    'behavior_server')
    bt_navigator        = nav2_node('nav2_bt_navigator', 'bt_navigator')
    waypoint_follower   = nav2_node('nav2_waypoint_follower', 'waypoint_follower')
    velocity_smoother   = nav2_node('nav2_velocity_smoother', 'velocity_smoother')
    collision_monitor   = nav2_node('nav2_collision_monitor', 'collision_monitor')

    # ── Lifecycle managers (split so map frame is independent of Nav2) ──
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_slam', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['slam_toolbox'],
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_node,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_slam,
        lifecycle_manager,
    ])
