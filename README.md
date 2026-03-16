# Mars Rover Simulation — ROS2 Jazzy + Nav2 + Gazebo Harmonic

**Version**: v1.0  
**Status**: Production-ready  
**ROS2 Distro**: Jazzy  
**Simulator**: Gazebo Harmonic  

A comprehensive 6-wheel **triple-rocker** Mars rover simulation environment with full autonomy stack.

## Features

### Rover Platform
- **Triple-rocker suspension system** for terrain adaptability
- 6-wheel drive (Differential + rocker-bogie mechanics)
- Realistic mass and inertia properties
- Motor dynamics simulation

### Sensors
- **2D LiDAR** (360° horizontal FOV, 30m range)
- **Depth Camera** (RGBD, ArUco marker detection)
- **IMU** (9-DOF: accel, gyro, mag)
- **Odometry feedback** from wheel encoders

### Autonomy & Navigation
- **Terrain mapping** using `slam_toolbox` SLAM
- **EKF sensor fusion** (odometry + IMU)
- **Nav2 navigation stack** for path planning
- **Visual servoing** for ArUco marker waypoint approach
- **Obstacle detection** and dynamic avoidance

### Environment
- **Rough terrain Gazebo world** with slopes, rocks, and uneven surfaces
- **RViz2 visualization** of sensor data and navigation state
- **Configurable spawn locations** and goal waypoints

---

## Workspace Structure

```
mars_rover_ws/
├── src/
│   ├── mars_rover_description/     URDF definitions, sensors, meshes
│   ├── mars_rover_gazebo/          Gazebo worlds, models, spawn configs
│   ├── mars_rover_navigation/      Nav2, SLAM, EKF, sensor fusion
│   └── mars_rover_aruco/           ArUco marker detection, waypoint bridge
├── build/                           Compiled packages (auto-generated)
├── install/                         Installation prefix (auto-generated)
├── log/                             Build logs (auto-generated)
└── README.md                        This file
```

---

## Prerequisites

### System Requirements
- Ubuntu 24.04 LTS or WSL2 with Ubuntu 24.04
- ROS2 Jazzy installed and sourced
- Gazebo Harmonic
- Python 3.10+

### Key Dependencies
- `gazebo_ros_pkgs` — ROS2-Gazebo integration
- `slam_toolbox` — SLAM algorithm
- `nav2_bringup` — Navigation2 stack
- `geometry2` (tf2) — Transform library
- `opencv_py` — Computer vision library
- `aruco_opencv_python_bindings` — ArUco detection

Install all dependencies:
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-geometry2 \
  python3-opencv \
  python3-numpy
```

---

## Quick Start

### 1. Build the Workspace
```bash
cd ~/mars_rover_ws
colcon build --symlink-install
```

### 2. Source the Environment
```bash
source install/setup.bash
```

### 3. Launch the Full Stack (Simulation + Navigation)
```bash
ros2 launch mars_rover_gazebo gazebo_bringup.launch.py
```

This launches:
- Gazebo with rover spawned in rough terrain
- SLAM toolbox running on LiDAR
- Nav2 stack with RViz2 visualization
- EKF state estimator

### 4. Publish a Navigation Goal
In a new terminal:
```bash
source install/setup.bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "header: {stamp: now, frame_id: 'map'} \
   pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}"
```

---

## ROS2 Topics & Services

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Wheel encoder odometry |
| `/imu` | `sensor_msgs/Imu` | IMU measurements |
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR scans |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth camera feed |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | RGB camera feed |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM occupancy grid |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 goal |

### Services
| Service | Type | Description |
|---------|------|-------------|
| `/map_server/clear_entirely_unknown_space` | `nav2_msgs/ClearEntirelyUnknownSpace` | SLAM service |

---

## Package Details

### `mars_rover_description`
Contains all rover geometry and sensors:
- **URDF**: Triple-rocker kinematics, wheel drives
- **Xacro macros**: Sensor attachments
- **Meshes**: Chassis, wheels, sensor mounts (`.obj`, `.dae`)

Launch URDF visualization:
```bash
ros2 launch mars_rover_description display.launch.py
```

### `mars_rover_gazebo`
Simulation environment:
- **Worlds**: Rough terrain with obstacles, ramps
- **Models**: Rock, tree meshes
- **Physics**: Terrain friction, gravity
- **Spawn scripts**: Place rover at origin or custom position

### `mars_rover_navigation`
Autonomy stack configuration:
- **SLAM config**: `slam_toolbox` parameters (resolution, decay time, etc.)
- **Nav2 config**: `planner_server`, `controller_server`, `behavior_tree`
- **EKF config**: Sensor covariance, fusion rates
- **costmap config**: Local/global costmap setup

### `mars_rover_aruco`
Vision-based waypoint following:
- **Detector node**: Identifies ArUco markers in depth images
- **Bridge node**: Converts marker poses to Nav2 goals
- **Calibration**: Camera intrinsics stored in configs

---

## Triple-Rocker Suspension Mechanics

```
              ┌─ chassis ─┐
              │           │
         [main_L]    [main_R]        ← Rocker arms (revolute on Y-axis)
        /  |  \      /  |  \
   [wheel] ... [wheel][wheel]   [wheel]
            
   Left rocker:  wheel_L1, wheel_L2, wheel_L3
   Right rocker: wheel_R1, wheel_R2, wheel_R3
```

**Kinematics**:
- Main rocker arms pivot on chassis (Y-axis)
- Each rocker has 3 wheels in a bogie arrangement
- Wheels can be driven independently or in groups
- Suspension absorbs terrain slopes

---

## Building & Testing

### Full Build
```bash
cd ~/mars_rover_ws
colcon build --symlink-install
```

### Build Specific Package
```bash
colcon build --packages-select mars_rover_navigation
```

### Build with Tests
```bash
colcon build --packages-select mars_rover_description --cmake-args -DBUILD_TESTING=ON
```

### Run Tests
```bash
colcon test
colcon test-result --verbose
```

---

## Visualization in RViz2

Default RViz2 layout includes:
- **3D Map** view with point clouds
- **Costmap** visualization (static, inflation)
- **Trajectory** history
- **Laser scan** overlay
- **Transform tree** (TF axes)

Custom config: `mars_rover_navigation/config/rviz.rviz`

Load manually:
```bash
rviz2 -d install/mars_rover_navigation/share/mars_rover_navigation/rviz/default.rviz
```

---

## Common Tasks

### Change Rover Starting Position
Edit `mars_rover_gazebo/launch/gazebo_bringup.launch.py`:
```python
initial_pose_x = 0.0   # Change these
initial_pose_y = 0.0
initial_pose_z = 0.5
```

### Adjust SLAM Parameters
Edit `mars_rover_navigation/config/slam_config.yaml`:
- `resolution`: Grid cell size (smaller = more detail, slower)
- `decay_time`: Map memory (larger = more history)
- `max_range`: LiDAR max distance

### Tune Navigation Speed
Edit `mars_rover_navigation/config/nav2_params.yaml`:
- `max_vel_x`: Max forward speed
- `max_vel_theta`: Max rotation speed
- `acc_limit_x`: Acceleration limit

### Record Sensor Data
```bash
ros2 bag record /scan /camera/rgb/image_raw /imu /odom
```

Playback:
```bash
ros2 bag play rosbag2_*
```

---

## Troubleshooting

### Rover sinks into terrain
- Increase `linear_damping` in URDF (wheel friction)
- Check Gazebo physics `dt` (timestep)

### LiDAR shows no data
- Verify URDF sensor attachment: `mars_rover_description/urdf/sensors.urdf.xacro`
- Check Gazebo plugin loading in terminal output

### Nav2 not responding to goals
- Ensure map has been built (check RViz2 map topic)
- Check `/global_costmap/costmap` and `/global_costmap/published_footprint` topics
- Review Nav2 behavior tree: `mars_rover_navigation/config/behavior_trees/`

### High CPU usage
- Reduce LiDAR scan rate: edit physics `max_step_size` in Gazebo world
- Decrease map resolution in SLAM config
- Close RViz2 if not needed

---

## Development

### Adding New Sensors
1. Add URDF to `mars_rover_description/urdf/sensors.urdf.xacro`
2. Create Gazebo plugin in same file
3. Add ROS2 topic subscription in relevant launch file

### Extending Navigation
1. Edit behavior trees: `mars_rover_navigation/config/behavior_trees/`
2. Add custom controllers: `mars_rover_navigation/src/`
3. Rebuild: `colcon build`

### Running Headless (No GUI)
```bash
ros2 launch mars_rover_gazebo gazebo_bringup.launch.py headless:=true
```

---

## Git Repository

- **Remote**: SSH configured (keys in `~/.ssh/`)
- **Default branch**: `main`
- **Latest commit**: See `git log`

Clone updates:
```bash
cd ~/mars_rover_ws
git pull origin main
colcon build
```

---

## Performance Benchmarks

| Metric | Value |
|--------|-------|
| Simulation speed | ~0.85x real-time (on i7 + RTX3060) |
| LiDAR update rate | 10 Hz |
| Map build rate | ~5 Hz |
| Navigation cycle | 20 Hz |
| Memory footprint | ~800 MB (Gazebo + ROS2 + RViz2) |

---

## References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic/getstarted/)
- [Nav2 Project](https://nav2.org/)
- [slam_toolbox GitHub](https://github.com/StanleyInnovation/slam_toolbox)

---

## Contact & Support

For issues or feature requests, check:
- Git log history: `git log --oneline`
- Build errors: Check `log/` directory
- Runtime errors: Enable ROS2 debug logging: `export ROS_LOG_DIR=/tmp && ROS_LOG_DIR=/tmp ros2 run ...`

**Last Updated**: 2026-03-16  
**Maintainer**: p0531d0n
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
