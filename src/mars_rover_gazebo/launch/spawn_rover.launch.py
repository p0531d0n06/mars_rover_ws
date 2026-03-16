"""
Launch Gazebo Harmonic with the Mars terrain world and spawn the rover.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('mars_rover_description')
    pkg_gazebo      = get_package_share_directory('mars_rover_gazebo')
    pkg_nav         = get_package_share_directory('mars_rover_navigation')

    xacro_file  = os.path.join(pkg_description, 'urdf', 'mars_rover.urdf.xacro')
    world_file  = os.path.join(pkg_gazebo, 'worlds', 'mars_terrain.sdf')
    ekf_params  = os.path.join(pkg_nav, 'config', 'ekf.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── Arguments ─────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Gazebo Harmonic — full (server + GUI via WSLg) ────────────────
    # --headless-rendering enables EGL off-screen rendering for camera
    # sensors on WSL2 (no /dev/dri) while the GUI still opens via WSLg.
    # Sensors plugin uses ogre (v1); ogre2 render thread hangs on WSL2.
    import os as _os
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--headless-rendering', world_file],
        output='screen',
        additional_env={
            'DISPLAY':                    _os.environ.get('DISPLAY', ':0'),
            'LIBGL_ALWAYS_SOFTWARE':      '1',
            'MESA_GL_VERSION_OVERRIDE':   '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
        },
    )

    # ── Robot State Publisher ──────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Spawn rover into Gazebo ────────────────────────────────────────
    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mars_rover',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.8',
        ],
        output='screen',
    )

    # ── ROS ↔ Gazebo Bridge ────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel: ROS → Gazebo (collision_monitor final output)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # odometry: Gazebo → ROS (dormant; swerve controller publishes /odom directly)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # tf: Gazebo → ROS
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # LiDAR — LaserScan (horizontal mid-plane, used by slam_toolbox)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # LiDAR — PointCloud2 (full 3D from 16-channel lidar)
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Front RGB camera
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Simulation clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # ── Swerve drive: per-wheel steer (rad) and drive (rad/s) ──
            '/gz/steer/left_front@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/steer/right_front@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/steer/left_mid@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/steer/right_mid@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/steer/left_rear@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/steer/right_rear@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/drive/left_front@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/drive/right_front@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/drive/left_mid@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/drive/right_mid@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/drive/left_rear@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/drive/right_rear@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # ── Swerve drive controller ────────────────────────────────────────
    swerve_controller = Node(
        package='mars_rover_gazebo',
        executable='swerve_drive_controller.py',
        name='swerve_drive_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── RViz2 for robot/map/scan visualisation ─────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(pkg_gazebo, 'config', 'mars_rover.rviz')]
                  if os.path.exists(os.path.join(pkg_gazebo, 'config', 'mars_rover.rviz'))
                  else [],
    )

    # ── Static TF: bridge Gazebo sensor frame → URDF frame ───────────
    # Gazebo names the sensor frame <model>/<link>/<sensor> using its
    # internal path.  The scan message has frame_id
    # "mars_rover/base_footprint/lidar" but robot_state_publisher only
    # knows "lidar_link".  This zero-offset static transform lets
    # slam_toolbox look up the transform chain correctly.
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'lidar_link', 'mars_rover/base_footprint/lidar'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── EKF — started here (with Gazebo) not with the delayed Nav2 stack ─
    # Publishing odom→base_footprint TF immediately ensures slam_toolbox
    # can look up sensor transforms from the very first scan, preventing
    # "timestamp earlier than TF cache" drops and message-filter queue overflow.
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom_ekf')],
    )

    return LaunchDescription([
        declare_use_sim_time,
        gz_sim,
        robot_state_pub,
        spawn_rover,
        ros_gz_bridge,
        swerve_controller,
        lidar_frame_bridge,
        ekf_node,
        rviz,
    ])
