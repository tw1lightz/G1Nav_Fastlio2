# Copilot instructions for HongTu

## Big picture (where to look first)
- This repo combines:
  - ROS1 (Noetic) catkin workspace for SLAM + 2D navigation: `G1Nav2D/`
  - Voice/MCP tooling that triggers navigation scripts: `PythonProject/py-xiaozhi-main/`
  - Robot control SDK (Unitree): `unitree_sdk2_python/`

## ROS workspace layout (G1Nav2D)
- `G1Nav2D/` is a catkin workspace (top-level CMake is catkin’s toplevel).
- Key packages and their roles:
  - `G1Nav2D/src/fastlio2` (package name `fastlio`): mapping (`map_builder_node`) + localization (`localizer_node`) + loop closure (GTSAM).
  - `G1Nav2D/src/livox_ros_driver2-master`: MID360 LiDAR driver.
  - `G1Nav2D/src/movebase` (package name `xju_pnc`): `move_base` + TEB planner params.
  - `G1Nav2D/src/tool`: topic/frame utilities (pointcloud frame conversion, downsample PCD, etc.).
  - `G1Nav2D/src/pointcloud_to_laserscan`: generates `LaserScan` for move_base from a pointcloud.
  - `G1Nav2D/src/velocity_smoother_ema`: smooths `cmd_vel`.

## Build & run (as documented in this repo)
- Primary runtime environment is Ubuntu + ROS Noetic (catkin). This repo may be edited on Windows, but the SLAM/nav stack commands below assume Ubuntu.
- LiDAR SDK prerequisite (Livox-SDK2) is required before building.
- Build driver if needed:
  - `cd G1Nav2D/src/livox_ros_driver2-master && ./build.sh ROS1`
- Build catkin workspace:
  - `cd G1Nav2D && catkin_make`
- Runtime:
  - `source G1Nav2D/devel/setup.bash`
  - Mapping: `roslaunch fastlio mapping.launch` (see `G1Nav2D/src/fastlio2/launch/mapping.launch`)
  - Navigation/localization: `roslaunch fastlio navigation.launch` (see `G1Nav2D/src/fastlio2/launch/navigation.launch`)

## Parameters, frames, topics (project-specific)
- `fastlio` defaults (topics/frames) are set via YAML:
  - Mapping: `G1Nav2D/src/fastlio2/config/mapping.yaml`
  - Localization: `G1Nav2D/src/fastlio2/config/localize.yaml`
- LiDAR → base transform is read from global params in `getLidar2BaseFromParam()`:
  - Params: `/lidar2base_xyz` and `/lidar2base_rpy` (see `G1Nav2D/src/fastlio2/src/map_builder_node.cpp`)
- `navigation.launch` composes the pipeline:
  - Loads 2D map via map_server (`G1Nav2D/src/fastlio2/launch/gridmap_load.launch`) publishing `/map_2d`.
  - Converts `velodyne_points` into `/base_link_cloud` via `tool/body2any_pointcloud.launch`.
  - Converts `/base_link_cloud` → `scan` via `pointcloud_to_laserscan` (`G1Nav2D/src/pointcloud_to_laserscan/launch/point_to_scan.launch`).
  - Runs `move_base` with TEB (`G1Nav2D/src/movebase/launch/move_base.launch`, params under `G1Nav2D/src/movebase/param/`).

## Topic/name sanity checks (when unsure)
- If topic naming is unclear on a robot, verify the live graph instead of assuming:
  - `rostopic list`
  - `rostopic info /velodyne_points` and `rostopic echo -n1 /velodyne_points`
  - `rostopic info /base_link_cloud` and `rostopic info /scan`
- When you change any pointcloud/scan topic name, update all of these consistently:
  - `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` publishers (e.g. `velodyne_points`, `slam_odom`)
  - `G1Nav2D/src/tool/launch/body2any_pointcloud.launch` remaps (`velodyne_points` → `*_cloud`)
  - `G1Nav2D/src/pointcloud_to_laserscan/launch/point_to_scan.launch` (`cloud_in`)
  - `G1Nav2D/src/movebase/param/costmap_params.yaml` (LaserScan topic `scan`, PointCloud2 topic currently `vlp_points`)

## Map saving conventions (fastlio)
- There is a ROS service `save_map` (advertised as `save_map` in the node, called as `/save_map`):
  - Example: `rosservice call /save_map "{save_path: '/abs/path/map.pcd', resolution: 0.0}"`
  - Note: `resolution` is currently unused (see `G1Nav2D/src/fastlio2/README.md`).
- The mapping node also auto-saves on SIGINT using hard-coded paths:
  - Update paths in `main()` of `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` (`g_map_path`, `g_ground_map_path`, `g_keyposes_path`).

## Voice → navigation integration (PythonProject)
- Voice keyword routing is implemented by string checks in `PythonProject/py-xiaozhi-main/src/application.py` (e.g. “电梯/楼梯” triggers an MCP tool call).
- MCP tool registration happens in `PythonProject/py-xiaozhi-main/src/mcp/mcp_server.py` via `get_*_manager().init_tools(...)`.
- Navigation-trigger tools often run external scripts with hard-coded paths:
  - `PythonProject/py-xiaozhi-main/src/mcp/tools/daohang_dianti/tools.py` (`python_path` + `script_path`)
  - `PythonProject/daohang/daohang-dianti.py` spawns a target nav-point script (also hard-coded)
  - When changing behavior, prefer adjusting these paths and the target `PythonProject/point_nav/*.py` scripts rather than editing core async/audio code.

## Editing guidelines for agents
- Prefer changing ROS params/launch/YAML over hard-coding new constants in C++.
- Keep topic and frame names consistent with the existing pipeline (`/livox/imu`, `/livox/lidar`, `velodyne_points`, `/base_link_cloud`, `/map_2d`).
- Treat `G1Nav2D/src/livox_ros_driver2-master` as vendored upstream: modify its `config/*.json` / launch files first.
