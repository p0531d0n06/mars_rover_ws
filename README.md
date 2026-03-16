# Mars Rover v3 — Autonomous Mission Planning & Hardware Integration

**Version**: v3.0  
**Status**: Stable  
**ROS2 Distro**: Jazzy  
**Simulator**: Gazebo Harmonic  
**Base**: mars_rover_ws_v2 with mission planning

Production-ready autonomous rover with mission planning, behavior trees, and hardware-in-the-loop simulation ready for real rover deployment.

## What's New in v3

### Mission Planning & Autonomy
- **Behavior trees** for complex mission sequences
- **Mission specification** language (YAML-based)
- **Conditional execution** based on sensor feedback
- **Recovery behaviors** for stuck/error states
- **Multi-objective optimization** (minimize time/energy)

### Hardware Integration
- **Hardware abstraction layer** for motor controllers
- **Real wheel encoder** integration (simulation + real)
- **IMU calibration routines** (bias, scale factor)
- **Camera exposure adjustment** algorithms
- **Power management** monitoring and alerting

### Robustness & Safety
- **Watchdog timers** for critical subsystems
- **Fault detection** and automatic failsafe
- **Graceful degradation** when sensors fail
- **Emergency stop** protocol with safe shutdown
- **Parameter validation** on startup

### Simulation-to-Reality Transfer
- **Domain randomization** for robust learning
- **Sensor noise models** matching real hardware
- **Physics parameters** calibrated to real rover
- **Friction models** for real terrain surface
- **Cable/connector tension** simulation

---

## Workspace Structure

```
mars_rover_ws_v3/
├── src/
│   ├── mars_rover_description/           Hardware models + CAD imports
│   ├── mars_rover_gazebo/                Physics-accurate worlds
│   ├── mars_rover_navigation/            Proven Nav2 config (v2)
│   ├── mars_rover_aruco/                 Stable vision node
│   ├── mars_rover_terrain_analysis/      Terrain classification (v2)
│   ├── mars_rover_missions/              NEW: Mission planning
│   ├── mars_rover_behaviors/             NEW: BT-based behaviors
│   ├── mars_rover_hardware/              NEW: Hardware drivers
│   └── mars_rover_safety/                NEW: Watchdog + failsafe
├── build/
├── install/
├── log/
├── missions/                             Mission definition files (YAML)
└── README.md
```

---

## Quick Start

### Build
```bash
cd ~/mars_rover_ws_v3
colcon build --symlink-install
```

### Simulation Mode

**Launch full stack:**
```bash
source install/setup.bash
ros2 launch mars_rover_gazebo gazebo_bringup.launch.py use_sim:=true
```

**Run a mission:**
```bash
ros2 run mars_rover_missions mission_executor \
  --mission_file missions/demo_exploration.yaml
```

### Pre-Flight Checklist
```bash
ros2 run mars_rover_missions mission_validator \
  --mission_file missions/demo_exploration.yaml
```

---

## Key Packages

### `mars_rover_missions`
Mission specification and execution engine using YAML format with real-time replanning and validation.

### `mars_rover_behaviors`
Behavior tree implementation using py_trees for complex autonomous sequences.

### `mars_rover_hardware`
Hardware abstraction layer supporting both simulation and real rover operation.

### `mars_rover_safety`
System monitoring with watchdog timers, fault detection, and automatic failsafe.

---

## Prerequisites

ROS2 Jazzy + Gazebo Harmonic + Python 3.10, plus:
```bash
sudo apt install -y \
  ros-jazzy-py-trees \
  ros-jazzy-py-trees-ros \
  ros-jazzy-diagnostic-aggregator \
  python3-pyyaml \
  python3-colorama
```

---

## Configuration

### Adjust Safety Limits
Edit `mars_rover_safety/config/safety_params.yaml`:
- `watchdog_timeout_sec`: Default 1.0s
- `battery_alert_thresholds`: [0.30, 0.15, 0.05]
- `motor_temp_limit_c`: Default 60°C
- `cpu_temp_limit_c`: Default 75°C

### Create Custom Missions
Place YAML files in `missions/` directory:
```yaml
version: 1.0
name: "Custom Mission"
waypoints:
  - {x: 1.0, y: 0.0}
  - {x: 2.0, y: 1.0}
```

---

## Running Missions

### Execute Mission
```bash
ros2 run mars_rover_missions mission_executor \
  --mission_file missions/demo_exploration.yaml \
  --log_directory ./mission_logs
```

### Monitor System Health
```bash
ros2 run mars_rover_safety safety_monitor --verbose
ros2 topic echo /mission_status
```

### Post-Mission Analysis
```bash
ros2 run mars_rover_missions mission_analyzer \
  --log_file mission_logs/2026-03-16-mission-*.log
```

---

## Hardware Integration

### Connect Real Rover
1. SSH into rover and launch drivers:
   ```bash
   ssh rover@robot.local
   source ~/mars_rover_hw/install/setup.bash
   ros2 launch mars_rover_hardware rover_bringup.launch.py
   ```

2. From development machine:
   ```bash
   export ROS_DOMAIN_ID=0
   ros2 run mars_rover_missions mission_executor \
     --mission_file missions/demo_exploration.yaml
   ```

### Calibration Procedures

**IMU calibration:**
```bash
ros2 run mars_rover_hardware imu_calibrator --duration 30
```

**Camera intrinsics:**
```bash
ros2 run mars_rover_hardware camera_calibrator
```

**Wheel encoders:**
```bash
ros2 run mars_rover_hardware encoder_calibrator --distance 1.0
```

---

## Benchmarks

| Metric | Value |
|--------|-------|
| Simulation speed | ~0.75x real-time |
| Mission planning latency | 50-100 ms |
| Behavior tree cycle | 10 Hz |
| Safety check frequency | 1 Hz |
| Memory footprint | ~1.1 GB |

---

## Git Repository

- **Base commit**: v2.0 (mars_rover_ws_v2)
- **Branch**: `feature/mission-autonomy`
- **Status**: Production-ready

Push changes:
```bash
cd ~/mars_rover_ws_v3
git commit -am "Feature: Add custom mission"
git push -u origin feature/mission-autonomy
```

---

## Deployment Checklist

Before mission deployment:
- [ ] Pre-flight validation passes
- [ ] Battery fully charged
- [ ] All sensors calibrated
- [ ] Network connectivity verified
- [ ] Emergency stop tested
- [ ] Recovery behaviors reviewed

---

## References

- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [ROS2 Mission Planning](https://docs.ros.org/en/jazzy/)
- [Behavior Trees in Robotics](https://arxiv.org/abs/1709.00050)

---

**Last Updated**: 2026-03-16  
**Maintainer**: p0531d0n  
**SSH Keys**: Configured in `~/.ssh/`
