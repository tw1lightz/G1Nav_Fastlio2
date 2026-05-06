# External Integrations

**Analysis Date:** 2026-05-06

## APIs & External Services

**AI / Voice:**
- OpenAI API - LLM-powered voice interaction in the py-xiaozhi voice assistant
  - SDK: `openai` Python package (`PythonProject/py-xiaozhi-main/requirements.txt`)
  - Usage: Natural language understanding for voice-based navigation commands
  - Auth: API key configured in py-xiaozhi application code (`PythonProject/py-xiaozhi-main/src/application.py`)
- WebSocket TTS Service - Real-time text-to-speech streaming
  - SDK: `websockets` Python package
  - Purpose: Audio response generation for voice interaction

**MQTT:**
- MQTT Protocol - Potential IoT/messaging integration
  - SDK: `paho-mqtt` Python package
  - Present in requirements but no direct usage found in scanned code

## Hardware Sensors & Actuators

**LiDAR Sensor:**
- Livox MID360 - 360-degree 3D LiDAR, primary perception sensor
  - Driver: `livox_ros_driver2` (v1.0.0, MIT license, official Livox SDK)
  - Connection: Ethernet, static IP `192.168.123.120`
  - Host IP: `192.168.123.164`
  - Ports: 56100-56500 (cmd_data, push_msg, point_data, imu_data, log_data)
  - Topics published: `/livox/imu` (IMU data), `/livox/lidar` (point cloud)
  - Config: `G1Nav2D/src/livox_ros_driver2-master/config/MID360_config.json`
  - SDK requirement: Livox SDK2 (system-installed library)

**Robot Platform:**
- Unitree G1 Humanoid Robot
  - SDK: `unitree_sdk2_python` (`unitree_sdk2_python/` directory)
  - Communication: Cyclone DDS via UDP network
  - Usage: High-level control of G1 robot (walking, turning, navigation)
  - Script: `unitree_sdk2_python/example/g1/high_level/g1_control.py`
  - Network: Uses a network interface (e.g., `eth0`) for DDS communication
  - Python package: configured via `pyproject.toml` in `unitree_sdk2_python/`

## Data Storage

**Databases:**
- None detected. No SQL or NoSQL database dependencies.

**File Storage:**
- Local filesystem only - Map data stored as PCD files (point cloud) and PGM/YAML files (2D grid maps)
  - Point cloud maps: `.pcd` files (e.g., `map.pcd`, `test2.pcd`, `ground_test3.pcd`)
  - 2D occupancy maps: `.pgm` + `.yaml` pairs (e.g., `map.pgm`, `map_fix.pgm`)
  - Map save/load handled by `map_server` (for 2D) and PCL I/O (for 3D PCD)
  - Map save command: `rosrun map_server map_saver map:=/projected_map -f <path>`

**Caching:**
- None detected.

## Authentication & Identity

**Auth Provider:**
- None detected. No authentication system for robot or ROS topics.

## Monitoring & Observability

**Error Tracking:**
- None detected. No Sentry, Datadog, or similar integration.

**Logs:**
- ROS console output only (`output="screen"` in launch files)
- Standard ROS logging (`rosconsole` / `rosout`)
- No centralized log aggregation

## CI/CD & Deployment

**Hosting:**
- On-device deployment on Unitree G1 robot computer (likely NVIDIA Jetson or similar ARM platform)
- Docker container: `ros1_noetic_hongtu` (image based on `ros:noetic`)

**CI Pipeline:**
- None detected. No GitHub Actions, GitLab CI, or Jenkins configuration.

**Build Process:**
- Manual build via `catkin_make` commands:
  1. `./build.sh ROS1` (livox driver build script)
  2. Sequential `catkin_make` invocations for each package
  3. No Dockerfile found in repository (Docker image assumed pre-built)

## Environment Configuration

**Required config:**
- LiDAR IP address in `MID360_config.json` (default: `192.168.123.120`)
- Host IP addresses in `MID360_config.json` (default: `192.168.123.164`)
- LiDAR extrinsic calibration in `mapping.yaml` / `localize.yaml`
  - IMU extrinsic rotation: identity matrix
  - IMU extrinsic position: `[-0.011, -0.02329, 0.04412]`
- 2D map file path in `gridmap_load.launch` (default: `/home/unitree/HongTu/map.yaml`)
- PCD map path in `navigation.launch` (default: `/root/HongTu/test2.pcd`)
- Navigation target coordinates in `PythonProject/point_nav/point*.py` (hardcoded coordinates)
- Map save path in `G1Nav2D/src/fastlio2/src/map_builder_node.cpp`

**Secrets location:**
- No secrets file detected. OpenAI API key may be hardcoded in py-xiaozhi application code.

## Webhooks & Callbacks

**Incoming:**
- None detected.

**Outgoing:**
- None detected.

## ROS Topic Interface (Inter-Package Integration)

**Published Topics:**
| Topic | Type | Source | Consumers |
|-------|------|--------|-----------|
| `/livox/imu` | `sensor_msgs/Imu` | livox_ros_driver2 | fastlio map_builder/localizer |
| `/livox/lidar` | `sensor_msgs/PointCloud2` (CustomMsg) | livox_ros_driver2 | fastlio map_builder/localizer |
| `/slam_odom` | `nav_msgs/Odometry` | fastlio | move_base, teb_local_planner, velocity_smoother |
| `/projected_map` | `nav_msgs/OccupancyGrid` | fastlio | map_server (save), RViz |
| `/map_2d` | `nav_msgs/OccupancyGrid` | map_server | move_base costmap |
| `/body_cloud` | `sensor_msgs/PointCloud2` | tool/body2any_pointcloud | octomap_server, pointcloud_to_laserscan |
| `/base_link_cloud` | `sensor_msgs/PointCloud2` | tool/body2any_pointcloud | pointcloud_to_laserscan |
| `/scan` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan | move_base costmap (2D obstacle layer) |
| `/cmd_vel` | `geometry_msgs/Twist` | move_base | velocity_smoother_ema |
| `/cmd_vel_smooth` | `geometry_msgs/Twist` | velocity_smoother_ema | Unitree G1 robot controller |

**Service Interface:**
| Service | Type | Package | Purpose |
|---------|------|---------|---------|
| `SlamReLoc` | Custom | fastlio | Trigger relocalization with pose guess |
| `SaveMap` | Custom | fastlio | Save current map to PCD |
| `MapConvert` | Custom | fastlio | Convert map coordinate frame |
| `SlamHold` | Custom | fastlio | Pause/resume SLAM optimization |
| `SlamStart` | Custom | fastlio | Start SLAM process |
| `SlamRelocCheck` | Custom | fastlio | Check relocalization status |
| `xju_task` | Custom | xju_pnc | Task execution service |

---

*Integration audit: 2026-05-06*
