# AAMMR — AI-powered Autonomous Medical Mobile Robot

AAMMR is an autonomous mobile robot (AMR) platform built on **ROS 2 Humble**, combining LiDAR-based SLAM, Nav2 autonomous navigation, and a custom Arduino-based differential-drive motor controller. The long-term goal of the project is to extend this navigation stack with **UV-C disinfection logic**, enabling autonomous surface and airborne pathogen reduction in hospital and clinical environments.


## Overview

The robot uses a 2D LiDAR for simultaneous localization and mapping (SLAM) and for Nav2-based autonomous path planning, while an Arduino Mega handles low-level motor control, quadrature-encoder odometry, and a PID-based drive loop. A ROS 2 bridge node relays velocity commands to the Arduino and republishes odometry and TF data back into the ROS graph.

## Hardware

| Component | Details |
|---|---|
| Compute | Raspberry Pi-class SBC running Ubuntu 22.04.5 |
| LiDAR | LD19, connected via GPIO UART (`/dev/ttyAMA0`) |
| Motor controller | Arduino Mega (USB serial) |
| Motors | JGY-370 DC gear motors with quadrature encoders |
| Motor drivers | BTS7960 H-bridges |
| Drivetrain | Differential drive (wheelbase ≈ 0.165–0.175 m, wheel diameter ≈ 0.065 m) |

## Software stack

- **ROS 2 Humble**
- **SLAM Toolbox** (async, scan-matching mode; wheel odometry is treated as unreliable and excluded from the SLAM solution)
- **Nav2** (AMCL localization, planner, controller, behavior tree navigator, lifecycle-managed)
- **RViz2** for visualization
- Custom `amr_robot` ROS 2 package (`ament_python`)

## Repository structure

```
AAMMR/
├── src/
│   ├── amr_robot/                  # Main ROS 2 package
│   │   ├── amr_robot/
│   │   │   ├── arduino_bridge.py   # ROS2 <-> Arduino serial bridge node
│   │   │   └── robot/
│   │   │       └── robot.ino       # Arduino Mega firmware (PID + encoder odometry)
│   │   ├── config/
│   │   │   ├── nav2_params.yaml    # Nav2 stack configuration
│   │   │   ├── slam_config.yaml    # SLAM Toolbox configuration
│   │   │   └── nav2.rviz           # RViz2 layout
│   │   ├── launch/
│   │   │   ├── mapping.launch.py       # LiDAR + bridge + SLAM Toolbox + teleop
│   │   │   ├── nav2.launch.py          # Full localization + navigation stack (office map)
│   │   │   └── nav2_Entrance.launch.py # Same stack, defaulting to the Entrance map
│   │   ├── maps/                   # Saved occupancy-grid maps (.pgm/.yaml)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── test/                   # ament lint / style tests
│   └── examples/
└── README.md
```

## Getting started

### Prerequisites

- Ubuntu 22.04 with ROS 2 Humble installed
- Dependencies declared in `package.xml`: `rclpy`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `slam_toolbox`, `ldlidar_stl_ros2`, `teleop_twist_keyboard`, `nav2_bringup`, `nav2_amcl`, `nav2_map_server`, `topic_tools`, `nav2_lifecycle_manager`
- `pyserial` (used by `arduino_bridge.py`)
- LD19 LiDAR wired to the SBC's GPIO UART
- Arduino Mega flashed with the firmware in `amr_robot/robot/robot.ino`, connected via USB

### Build

```bash
mkdir -p ~/ros2_ws
cp -r AAMMR/src ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Add the workspace source line to `~/.bashrc` so the `amr_robot` package is discoverable in new shells:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Mapping a new environment

```bash
ros2 launch amr_robot mapping.launch.py
```

This brings up the LD19 LiDAR driver, the Arduino bridge (publishing high-covariance `/odom` and a continuous `odom→base_link` TF), SLAM Toolbox in scan-matching mode, and a keyboard teleop window for manually driving the robot while the map builds. Save the resulting map with `nav2_map_server`'s `map_saver_cli` once mapping is complete.

### Autonomous navigation

```bash
ros2 launch amr_robot nav2.launch.py
# or, for the alternate pre-saved map:
ros2 launch amr_robot nav2_Entrance.launch.py
```

Each launch file brings up the LiDAR, a `/scan` throttle (10 Hz), a static `base_link → base_laser` transform, the Arduino bridge, AMCL localization against the chosen saved map, the full Nav2 navigation stack, and RViz2, staggered with timers so each subsystem comes up only once its dependencies are ready.

To navigate against a different saved map, override the `map` launch argument:

```bash
ros2 launch amr_robot nav2.launch.py map:=/path/to/your_map.yaml
```

## Firmware (`robot.ino`)

The Arduino Mega firmware reads quadrature encoders on the left and right motors, runs independent PID loops on wheel speed (plus a heading-correction PID), and drives the motors through the BTS7960 H-bridges. It communicates with the ROS 2 bridge over a simple serial protocol:

| Command | Purpose |
|---|---|
| `CMD,vL,vR` | Velocity command from ROS 2. Triggers a 500 ms watchdog — if no `CMD` is received within that window, the robot stops automatically. |
| `S vL vR` | Manual/serial-monitor speed command. Does **not** trigger the ROS watchdog — use this for bench testing instead of `CMD`. |
| `G x y` | Go to a target position (basic point navigation). |
| `H deg` | Rotate to a target heading. |
| `P` | Print current odometry (`ODO,x,y,θ,vL,vR,encL,encR`). |
| `R` | Reset odometry. |

The bridge node (`arduino_bridge.py`) also applies per-wheel deadband compensation, lifting any nonzero `/cmd_vel`-derived wheel speed that falls below each motor's minimum moving speed (configurable via the `min_wheel_speed_left` / `min_wheel_speed_right` parameters), since Nav2's controller frequently commands speeds too small to overcome motor deadband.

## Design references

Emitter placement strategy, dwell-time logic, safety shutoff behavior, post-cycle ventilation, and shadow-zone handling for the planned UV-C disinfection layer are being derived from a clinical study of the Surfacide® Helios 254 nm UV-C multiemitter system (Fiscal-Baxin et al., *Pathogens*, 2026), mapping its operational protocols onto Nav2 autonomy states.


## License

MIT (see `package.xml`).
