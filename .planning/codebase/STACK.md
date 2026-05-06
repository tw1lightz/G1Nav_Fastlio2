# Technology Stack

**Analysis Date:** 2026-05-06

## Languages

**Primary:**
- C++14 / C++17 - All core ROS nodes for SLAM, navigation, and LiDAR driver
  - `G1Nav2D/src/fastlio2/` uses C++14 (`-std=c++14`)
  - `G1Nav2D/src/movebase/` uses C++17 (`-std=c++17`)
  - `G1Nav2D/src/pointcloud_to_laserscan/` uses C++11/14 (`cmake_minimum_required(VERSION 2.8.3)`)
  - `G1Nav2D/src/velocity_smoother_ema/` uses C++11 default

**Secondary:**
- Python 3.9 - 3.12 - Voice interaction system and utility scripts
  - `PythonProject/py-xiaozhi-main/` (voice assistant)
  - `G1Nav2D/src/fastlio2/scripts/slam_reloc.py` (SLAM relocalization)
  - `G1Nav2D/src/tool/scripts/` (teaching path recording/playback)
  - `G1Nav2D/client/` (robot control SDK wrappers)
  - `PythonProject/point_nav/` (coordinate navigation points)
  - `PythonProject/daohang/` (navigation dispatch scripts)
  - `visualize.py` (PCD visualization using Open3D)

## Runtime

**Environment:**
- ROS Noetic Ninjemys (ROS1) - Full ROS1 distribution on Ubuntu 20.04
- Deployed via Docker container (`ros1_noetic_hongtu` container image)

**Package Manager:**
- catkin (ROS1 build system) via `catkin_make`
- apt (system packages for ros-noetic-* dependencies)
- pip (Python dependencies in `PythonProject/py-xiaozhi-main/requirements.txt`)
- No package-lock / lockfile for Python dependencies

## Frameworks

**Core Robotics:**
- ROS Noetic (ROS1) - Robot Operating System middleware for all inter-process communication
- GTSAM 4.x - Factor graph optimization (ISAM2) for SLAM backend
  - `D:\G1_dev\G1Nav_Fastlio2\G1Nav2D\src\fastlio2\src\map_builder_node.cpp`: `#include <gtsam/nonlinear/ISAM2.h>`
- PCL 1.x (Point Cloud Library) - Point cloud processing, ICP registration, voxel grid filtering
- Eigen3 - Linear algebra for state estimation and transforms
- OpenMP - Multi-processing for parallel point cloud operations (x86 only)

**SLAM-Specific:**
- ikd-Tree - Incremental k-d tree for fast point cloud insertion and search
  - `G1Nav2D/src/fastlio2/include/ikd-Tree/ikd_Tree.cpp`
- IKFoM (Error-State Kalman Filter on Manifold) - Core filter implementation
  - `G1Nav2D/src/fastlio2/include/IKFoM_toolkit/esekfom/esekfom.hpp`
- FAST-LIO2 - Core LIO (LiDAR-Inertial Odometry) framework

**Navigation:**
- move_base - ROS navigation stack
- teb_local_planner - Timed Elastic Band local planner
- global_planner - ROS global planner
- costmap_2d - 2D costmap layers (static, obstacle, inflation)
- octomap_server - 3D occupancy mapping
- map_server - 2D grid map loading/saving

**Visualization / GUI:**
- RViz - ROS 3D visualization
- Qt5 (Core, Widgets) - RViz plugin for map editing
  - `G1Nav2D/src/ros_map_edit/` uses Qt5 for map editor tool
- OpenCV - Image processing for map editing (cv_bridge)

**Testing:**
- gtest - Available as test framework in package.xml templates, no actual tests found
- roslint - Linting for `pointcloud_to_laserscan` package
- No comprehensive test suite detected

**Voice/AI (Python):**
- openai 1.86.0 - LLM-based voice interaction
- Vosk 0.3.44 - Offline speech recognition
- websockets 11.0.3 - Real-time TTS streaming
- aiohttp 3.12.13 - Async HTTP for AI service calls
- PyQt5 5.15.11 - Voice assistant GUI

**Build/Dev:**
- CMake 3.0.2+ - Build system for all C++ packages
- catkin - ROS1 package build macros
- build.sh - Convenience build script for livox_ros_driver2

## Key Dependencies

**Critical:**
- `livox_ros_driver2` - Official Livox LiDAR ROS2/ROS1 driver (v1.0.0, MIT license)
- `gtsam` - Georgia Tech Smoothing and Mapping library for factor graph SLAM
- `pcl` - Point Cloud Library for all point cloud processing
- `eigen3` - Core linear algebra throughout the codebase

**Infrastructure:**
- `roscpp` / `rospy` - ROS client libraries for C++ and Python
- `tf2` / `tf2_ros` - ROS transform library
- `pcl_ros` - ROS-PCL bridge for point cloud messages
- `nav_msgs` / `geometry_msgs` / `sensor_msgs` / `std_msgs` - ROS message types
- `actionlib` - ROS action protocol for navigation goals

**Python (Voice System):**
- `openai==1.86.0` - LLM API client for voice interaction
- `vosk==0.3.44` - Offline speech recognition
- `websockets==11.0.3` - Real-time communication
- `paho-mqtt==2.1.0` - MQTT messaging
- `sounddevice>=0.4.4` - Audio capture and playback
- `numpy==1.26.4` - Numerical computation
- `requests==2.32.3` - HTTP API calls
- `aiohttp==3.12.13` - Async HTTP client/server
- `opencv-python-headless==4.11.0.86` - Image processing

## Configuration

**Environment:**
- ROS parameters loaded from YAML files at runtime via `<rosparam command="load" />`
- Livox LiDAR configured via JSON: `G1Nav2D/src/livox_ros_driver2-master/config/MID360_config.json`
- LiDAR IP: `192.168.123.120` (configurable), host IP: `192.168.123.164`
- Map paths passed as ROS launch arguments (PCD map, 2D YAML map)
- No `.env` files detected

**Key Config Files:**
- `G1Nav2D/src/fastlio2/config/mapping.yaml` - SLAM mapping parameters
- `G1Nav2D/src/fastlio2/config/localize.yaml` - Localization parameters
- `G1Nav2D/src/movebase/param/costmap_params.yaml` - Costmap (global + local) configuration
- `G1Nav2D/src/movebase/param/teb_local_planner_params.yaml` - TEB planner tuning
- `G1Nav2D/src/movebase/param/move_base_params.yaml` - move_base behavior
- `G1Nav2D/src/velocity_smoother_ema/param/smoother.yaml` - Velocity smoothing (alpha values)
- `G1Nav2D/src/livox_ros_driver2-master/config/MID360_config.json` - LiDAR network config

## Platform Requirements

**Development:**
- Ubuntu 20.04 (host or Docker container)
- ROS Noetic (ros-noetic-desktop-full or equivalent)
- Livox SDK2 (system-level library, installed via `make install`)
- `ros-noetic-teb-local-planner`, `ros-noetic-global-planner`, `ros-noetic-costmap-server` apt packages
- CMake, g++, Python 3.9+

**Production:**
- Unitree G1 humanoid robot
- Livox MID360 LiDAR sensor
- Docker container based on ROS Noetic image
- Network: Static IP configuration (LiDAR at 192.168.123.120, host at 192.168.123.164)

---

*Stack analysis: 2026-05-06*
