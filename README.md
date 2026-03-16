# Mars Rover Simulation — ROS2 Jazzy + Nav2 + Gazebo Harmonic

6-wheel **triple-rocker** Mars rover simulation with:
- Rough terrain Gazebo world
- 2D LiDAR → `slam_toolbox` terrain mapping
- Depth camera → ArUco marker waypoint detection
- EKF sensor fusion (odometry + IMU)
- Nav2 coarse navigation + visual servoing fine approach

---

## Workspace structure

```
mars_rover_ws/
└── src/
    ├── mars_rover_description/   URDF/Xacro + sensor definitions
    ├── mars_rover_gazebo/        Worlds, models, spawn launch
    ├── mars_rover_navigation/    Nav2, slam_toolbox, EKF configs
    └── mars_rover_aruco/         ArUco detector + waypoint bridge
```

---

## Triple-rocker suspension

```
         chassis
        /       \
  [main_L]     [main_R]        ← revolute (roll axis, Y)
  /  |  \      /  |  \
[FL][LM][RL]  [FR][RM][RR]
     ↑                ↑
  front/rear sub-rockers pivot from main rocker tips
  mid wheel sits on main rocker body
```

Each side: 1 main rocker + 2 sub-rockers + 3 wheels = **triple rocker**.
All 6 rocker joints are passive revolutes (no actuation), providing
passive terrain compliance without active suspension control.

---

## Prerequisites

```bash
# Install Nav2, slam_toolbox, robot_localization
sudo apt-get install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-image \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-vision-opencv \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  python3-opencv
```

---

## Build

```bash
cd ~/mars_rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Generate ArUco marker images (first-time only)

```bash
python3 ~/mars_rover_ws/src/mars_rover_gazebo/config/generate_aruco_markers.py
```

This writes `marker_0.png` … `marker_4.png` into
`mars_rover_gazebo/models/aruco_marker/`.

---

## Launch

### Full simulation (Gazebo + Nav2 + ArUco)
```bash
ros2 launch mars_rover_gazebo simulation.launch.py
```

### Individual components (for debugging)
```bash
# Gazebo + rover only
ros2 launch mars_rover_gazebo spawn_rover.launch.py

# URDF preview in RViz (no Gazebo)
ros2 launch mars_rover_description view_robot.launch.py

# Navigation stack only (requires Gazebo already running)
ros2 launch mars_rover_navigation navigation.launch.py

# ArUco nodes only
ros2 launch mars_rover_aruco aruco.launch.py
```

---

## How it works

### Mapping (slam_toolbox)
The 2D LiDAR publishes `/scan`. `slam_toolbox` builds a live occupancy-grid
map, publishing `/map` and the `map → odom` transform.

### Localisation (robot_localization EKF)
Fuses `/odom` (Gazebo diff-drive) + `/imu/data` into a smooth dead-reckoning
estimate. The `map → base_footprint` chain is:
`map → odom (slam_toolbox) → odom_ekf (robot_localization) → base_footprint`

### Waypoint detection (ArUco)
The `aruco_detector` node watches the depth camera. When a marker is
detected, it publishes the 3D pose in `/aruco/poses`. The
`aruco_waypoint_bridge` node:
1. **Coarse phase** — sends a `NavigateToPose` goal to Nav2 ~1.5 m from
   the marker using pre-configured approach coordinates.
2. **Fine phase** — once Nav2 succeeds, switches to proportional visual
   servoing (cmd_vel) aligned to the marker centre until within 0.4 m.

### Waypoint layout (world frame)

| ID | Name    | Position      |
|----|---------|---------------|
| 0  | Alpha   | (5, 0)        |
| 1  | Beta    | (5, 5)        |
| 2  | Gamma   | (0, 8)        |
| 3  | Delta   | (-5, 5)       |
| 4  | Epsilon | (-5, -5)      |

---

## Tuning

| File | What to tune |
|------|-------------|
| `nav2_params.yaml` | Speed, lookahead, goal tolerance |
| `slam_toolbox.yaml` | Map resolution, loop closure |
| `ekf.yaml` | Sensor noise covariances |
| `aruco_waypoints.yaml` | Marker sizes, approach distances, waypoint coords |

---

## Key topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR → SLAM |
| `/depth_camera/image_raw` | `sensor_msgs/Image` | Camera → ArUco |
| `/depth_camera/depth_image` | `sensor_msgs/Image` | Depth → ArUco |
| `/imu/data` | `sensor_msgs/Imu` | IMU → EKF |
| `/odom` | `nav_msgs/Odometry` | Gazebo → Nav2/EKF |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2/ArUco → rover |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM output |
| `/aruco/detections` | `std_msgs/String` (JSON) | Detected markers |
| `/aruco/poses` | `geometry_msgs/PoseArray` | 3D marker poses |

---

## References

- Base rover concept: [LeoRover/leo_simulator-ros2](https://github.com/LeoRover/leo_simulator-ros2) (Mars Yard worlds)
- ArUco detection pattern: [AIRLab-POLIMI/ros2-aruco-pose-estimation](https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation)
- Nav2 + ArUco docking pattern: [Vor7reX/ibt_ros2_autodocking](https://github.com/Vor7reX/ibt_ros2_autodocking)
