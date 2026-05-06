# Codebase Structure

**Analysis Date:** 2026-05-06

## Directory Layout

```
G1Nav_Fastlio2/
├── G1Nav2D/                        # Catkin workspace root
│   ├── .catkin_workspace           # Catkin workspace marker
│   ├── .vscode/settings.json       # VS Code C++ intellisense config (legacy Go2Nav2D paths)
│   ├── client/                     # Python ROS client for Unitree G1 robot
│   │   ├── YgClient.py             # ROS node: /cmd_vel_smooth -> DogControllerSDK
│   │   ├── DogControllerSDK.py     # HTTP SDK for Unitree G1 dog control
│   │   ├── constants.py            # Dog action enum (StandUp, Sit, Dance, etc.)
│   │   └── clientTrue.html         # Web-based dog control interface
│   └── src/                        # ROS packages (catkin source space)
│       ├── CMakeLists.txt          # Top-level catkin CMake (symlink to /opt/ros/noetic)
│       ├── fastlio2/               # FAST-LIO2 SLAM package (core SLAM system)
│       ├── livox_ros_driver2-master/ # Livox LiDAR ROS driver
│       ├── movebase/               # move_base + TEB navigation config
│       └── velocity_smoother_ema/  # EMA velocity smoothing filter
├── docs/                           # Project documentation and notes
│   ├── 文件读写常用命令速查.md     # Command cheat sheet (Chinese)
│   ├── 工作日志.txt               # Work log
│   ├── 经验总结.md                # Experience summary
│   └── todos.md                   # TODO list
├── *.pcd                          # PCD map files (at root, not committed to workspace)
├── README.md                      # Deployment and usage guide
└── .planning/codebase/            # Codebase analysis documents (this directory)
```

## Directory Purposes

**`G1Nav2D/client/`:**
- Purpose: ROS-to-robot bridge for Unitree G1 quadruped
- Contains: Python ROS nodes and SDK wrappers
- Key files:
  - `YgClient.py`: ROS node subscribing `/cmd_vel_smooth`, publishing velocity commands via DogControllerSDK
  - `DogControllerSDK.py`: HTTP-based client for the unitree robot API
  - `constants.py`: Action type enum (StandUp, Sit, Damp, Dance, etc.)

**`G1Nav2D/src/fastlio2/`:**
- Purpose: Core SLAM package -- FAST-LIO2 LiDAR-inertial odometry, loop closure, ICP localization, ground extraction
- Contains: C++ source, headers, launch files, config, services, scripts
- Subpackage structure:
  - `src/`: ROS node entry points
  - `src/lio_builder/`: Core ESIKF engine + IMU processor
  - `src/localizer/`: ICP-based relocalization
  - `include/lio_builder/`: LIOBuilder header
  - `include/localizer/`: IcpLocalizer header
  - `include/ikd-Tree/`: Incremental kd-tree implementation (forked)
  - `include/IKFoM_toolkit/`: IKFoM manifold math toolkit (submodule)
  - `launch/`: ROS launch files
  - `config/`: YAML parameter configs
  - `srv/`: ROS service definitions
  - `scripts/`: Python utility scripts
  - `rviz/`: RViz configuration files
  - `PCD/`: Default map storage directory

**`G1Nav2D/src/livox_ros_driver2-master/`:**
- Purpose: Livox LiDAR hardware driver for ROS (MID360 LiDAR)
- Contains: C++ driver source, hardware communication, config JSON, launch files
- Key subdirectories:
  - `src/`: Driver implementation (driver_node, lds_lidar, callbacks, comm)
  - `3rdparty/rapidjson/`: Bundled JSON parser (not a ROS dependency)
  - `launch_ROS1/`: ROS1-specific launch files (msg_MID360.launch, etc.)
  - `config/`: LiDAR hardware config JSON files

**`G1Nav2D/src/movebase/`:**
- Purpose: Navigation stack configuration (move_base + TEB local planner)
- Contains: Launch files, YAML parameter files, utility source
- Key files:
  - `launch/move_base.launch`: Launches move_base node with costmap/teb params
  - `param/costmap_params.yaml`: Costmap configuration
  - `param/teb_local_planner_params.yaml`: TEB planner tuning (speeds, tolerances, weights)
  - `param/move_base_params.yaml`: move_base general params
  - `param/costmap_converter_params.yaml`: Costmap conversion to obstacles
  - `param/recovery_behavior_params.yaml`: Recovery behavior config
  - `src/costmap_clear.cpp`: Small node to clear costmaps on /initialpose

**`G1Nav2D/src/velocity_smoother_ema/`:**
- Purpose: EMA-based velocity smoothing with stop timeout
- Contains: C++ ROS node, launch file, YAML params
- Key files:
  - `src/velocity_smoother_ema.cpp`: Main filter node (sub raw_cmd_vel, pub cmd_vel_smooth)
  - `include/velocity_smoother_ema/velocity_smoother_ema.hpp`: Header
  - `param/smoother.yaml`: alpha_v, alpha_w, topic names, rate, stop_counter

## Key File Locations

**Entry Points (ROS Nodes):**
- `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` -- MapBuilderROS (mapping mode with loop closure)
- `G1Nav2D/src/fastlio2/src/localizer_node.cpp` -- LocalizerROS (navigation mode with ICP relocalization)
- `G1Nav2D/src/fastlio2/src/temp_node.cpp` -- Minimal FAST-LIO2 test node
- `G1Nav2D/src/velocity_smoother_ema/src/velocity_smoother_ema.cpp` -- Velocity smoother
- `G1Nav2D/src/movebase/src/costmap_clear.cpp` -- Costmap clearer
- `G1Nav2D/client/YgClient.py` -- Robot control bridge

**Launch Files:**
- `G1Nav2D/src/fastlio2/launch/mapping.launch` -- Mapping mode launcher
- `G1Nav2D/src/fastlio2/launch/navigation.launch` -- Full navigation system launcher
- `G1Nav2D/src/fastlio2/launch/octomap_server.launch` -- 3D occupancy mapping
- `G1Nav2D/src/fastlio2/launch/gridmap_load.launch` -- 2D grid map server
- `G1Nav2D/src/movebase/launch/move_base.launch` -- move_base + TEB setup
- `G1Nav2D/src/velocity_smoother_ema/launch/velocity_smoother_ema.launch` -- Velocity smoother
- `G1Nav2D/src/livox_ros_driver2-master/launch_ROS1/msg_MID360.launch` -- Livox MID360 driver

**Configuration:**
- `G1Nav2D/src/fastlio2/config/mapping.yaml` -- Mapping params (LIO, loop closure, dynamic filter)
- `G1Nav2D/src/fastlio2/config/localize.yaml` -- Localization params (LIO, ICP localizer)
- `G1Nav2D/src/movebase/param/costmap_params.yaml` -- Costmap layers and parameters
- `G1Nav2D/src/movebase/param/teb_local_planner_params.yaml` -- TEB planner parameters
- `G1Nav2D/src/velocity_smoother_ema/param/smoother.yaml` -- EMA smoothing coefficients

**Core Logic:**
- `G1Nav2D/src/fastlio2/src/lio_builder/lio_builder.cpp` -- LIO engine (ESIKF, ikd-Tree, mapping loop)
- `G1Nav2D/src/fastlio2/src/lio_builder/imu_processor.cpp` -- IMU preintegration and initialization
- `G1Nav2D/src/fastlio2/src/localizer/icp_localizer.cpp` -- Multi-resolution ICP localization
- `G1Nav2D/src/fastlio2/include/ikd-Tree/ikd_Tree.h` -- Incremental kd-tree (point cloud map)
- `G1Nav2D/src/fastlio2/include/IKFoM_toolkit/esekfom/esekfom.hpp` -- ESIKF filter

**Service Definitions:**
- `G1Nav2D/src/fastlio2/srv/SlamReLoc.srv` -- Relocalization trigger (pose initial guess)
- `G1Nav2D/src/fastlio2/srv/SlamHold.srv` -- Pause SLAM
- `G1Nav2D/src/fastlio2/srv/SlamStart.srv` -- Resume SLAM
- `G1Nav2D/src/fastlio2/srv/SlamRelocCheck.srv` -- Check relocalization status
- `G1Nav2D/src/fastlio2/srv/MapConvert.srv` -- PCD map conversion
- `G1Nav2D/src/fastlio2/srv/SaveMap.srv` -- Save map to PCD
- `G1Nav2D/src/movebase/srv/xju_task.srv` -- Task service definition

**Scripts:**
- `G1Nav2D/src/fastlio2/scripts/slam_reloc.py` -- Bridge RViz 2D Pose Estimate to /slam_reloc

## Naming Conventions

**Files:**
- C++ source: `snake_case.cpp` (e.g., `map_builder_node.cpp`, `imu_processor.cpp`, `icp_localizer.cpp`, `velocity_smoother_ema.cpp`)
- C++ headers: `snake_case.h` or `snake_case.hpp` (e.g., `commons.h`, `lio_builder.h`, `imu_processor.h`, `velocity_smoother_ema.hpp`)
- Launch files: `snake_case.launch`
- Config files: `snake_case.yaml`
- Service files: `PascalCase.srv` (e.g., `SlamReLoc.srv`, `MapConvert.srv`)
- Python scripts: `snake_case.py` (e.g., `slam_reloc.py`, `YgClient.py`)
- ROS node names: `snake_case_node` (e.g., `map_builder_node`, `localizer_node`)

**Directories:**
- ROS packages: `snake_case` (e.g., `fastlio2`, `movebase`, `velocity_smoother_ema`)
- Library directories: `snake_case` (e.g., `lio_builder`, `localizer`, `ikd-Tree`)
- Configuration directories: `param`, `config`, `launch`

**Code:**
- Classes: `PascalCase` (e.g., `MapBuilderROS`, `LocalizerROS`, `LIOBuilder`, `LoopClosureThread`, `IcpLocalizer`, `IMUProcessor`)
- Structs: `PascalCase` (e.g., `SharedData`, `LoopPair`, `Pose6D`, `MeasureGroup`, `LivoxData`)
- Functions: `camelCase` (e.g., `addKeyPose`, `publishCloud`, `syncPackage`, `transformToWorld`, `addNorm`)
- Member variables: `snake_case_` with trailing underscore (e.g., `shared_data_`, `lio_builder_`, `ikdtree_`, `imu_processor_`)
- ROS topics: `snake_case` (e.g., `/slam_odom`, `/body_cloud`, `/cmd_vel_smooth`)
- TF frames: `snake_case` (e.g., `map`, `local`, `body`, `base_link`)
- ROS parameters: `snake_case` with namespace: e.g., `/lio_builder/det_range`, `/loop_closure/activate`

## Where to Add New Code

**New ROS Node:**
- C++ source: `G1Nav2D/src/<package>/src/<node_name>.cpp`
- Header (if needed): `G1Nav2D/src/<package>/include/<package>/<module>.h`
- CMake registration: `G1Nav2D/src/<package>/CMakeLists.txt` (add `add_executable` + `target_link_libraries`)
- Launch integration: `G1Nav2D/src/<package>/launch/<name>.launch`
- Config: `G1Nav2D/src/<package>/config/<name>.yaml`

**New Library/Module within fastlio2:**
- Implementation: `G1Nav2D/src/fastlio2/src/<module>/<name>.cpp`
- Header: `G1Nav2D/src/fastlio2/include/<module>/<name>.h`
- Add to `SRC_LIST` in `G1Nav2D/src/fastlio2/CMakeLists.txt` (line 98)

**New Service Definition:**
- SRV file: `G1Nav2D/src/fastlio2/srv/<Name>.srv`
- Add to `add_service_files(FILES ...)` in `G1Nav2D/src/fastlio2/CMakeLists.txt` (lines 62-70)

**New Python Script:**
- ROS script: `G1Nav2D/src/fastlio2/scripts/<name>.py`
- Register install: `catkin_install_python(PROGRAMS ...)` in CMakeLists.txt (lines 132-136)

**New Launch Configuration:**
- Launch file: `G1Nav2D/src/fastlio2/launch/<name>.launch`
- Config YAML: `G1Nav2D/src/fastlio2/config/<name>.yaml`
- If adding to navigation pipeline, include from `navigation.launch`

**New Parameter (fastlio2):**
- Add `nh_.param<T>("/namespace/key", variable, default)` in the `initParams()` function of the relevant node class
- Add to the appropriate YAML config file (`mapping.yaml` or `localize.yaml`)

**Utilities:**
- Shared helper functions (TF, odometry, PCL conversions): `G1Nav2D/src/fastlio2/src/commons.cpp` / `G1Nav2D/src/fastlio2/include/commons.h`

**Client-side code:**
- Python nodes: `G1Nav2D/client/<name>.py`
- Robot SDK: `G1Nav2D/client/DogControllerSDK.py`

## Special Directories

**`G1Nav2D/src/fastlio2/include/ikd-Tree/`:**
- Purpose: Incremental kd-tree implementation for point cloud map management
- Contains: `ikd_Tree.h` (header-only with template implementation), `ikd_Tree.cpp` (compiled separately)
- Notes: Forked from FAST-LIO2 upstream; includes `.cpp` in include directory (unconventional)
- Committed: Yes

**`G1Nav2D/src/fastlio2/include/IKFoM_toolkit/`:**
- Purpose: IKFoM (Iterated Kalman Filter on Manifolds) mathematical toolkit
- Contains: ESIKF implementation (`esekfom/`), manifold types (`mtk/` -- SO3, S2, vect)
- Notes: Header-only template library; vendored dependency
- Committed: Yes

**`G1Nav2D/src/livox_ros_driver2-master/3rdparty/rapidjson/`:**
- Purpose: Bundled rapidjson library for Livox driver config parsing
- Committed: Yes

**`G1Nav2D/src/fastlio2/PCD/`:**
- Purpose: Default directory for saved map PCD files
- Contains: `map.pcd`, `ground_map.pcd` (default save location)
- Notes: Path is configurable via source code edit in `map_builder_node.cpp`; save path can be changed during runtime through `/save_map` service call
- Committed: Yes (PCD files exist in repo)

**`G1Nav2D/src/fastlio2/path/`:**
- Purpose: Saved keyframe trajectory files
- Contains: `key_poses.txt` (saved keyframe poses)
- Notes: Used for auto-save on SIGINT
- Committed: Yes

---

*Structure analysis: 2026-05-06*
