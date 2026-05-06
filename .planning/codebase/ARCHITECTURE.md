<!-- refreshed: 2026-05-06 -->
# Architecture

**Analysis Date:** 2026-05-06

## System Overview

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                        UNITREE G1 QUADRUPED ROBOT                              │
│                                                                                │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                        Client Layer (Python)                              │  │
│  │  ┌─────────────────────┐    ┌─────────────────────────────────────────┐  │  │
│  │  │ YgClient.py          │    │ DogControllerSDK.py                     │  │  │
│  │  │ CmdVelController     │───>│ HTTP SDK -> robot IP:192.168.58.126    │  │  │
│  │  │ /cmd_vel_smooth sub  │    └─────────────────────────────────────────┘  │  │
│  │  └─────────────────────┘                                                 │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
│                                    │                                            │
│                                    ▼                                            │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                     ROS Navigation Stack (ROS1 Noetic)                    │  │
│  │                                                                           │  │
│  │  ┌──────────────────────┐    ┌─────────────────────┐                     │  │
│  │  │ velocity_smoother_ema│    │ move_base (TEB)      │                     │  │
│  │  │ raw_cmd_vel ───────────>│ /cmd_vel               │                     │  │
│  │  │ cmd_vel_smooth out   │    │ costmap_clear         │                     │  │
│  │  └──────────────────────┘    └─────────────────────┘                     │  │
│  │                                    │                                       │  │
│  │                                    ▼                                       │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐     │  │
│  │  │                    SLAM & Localization                           │     │  │
│  │  │                                                                  │     │  │
│  │  │  ┌──────────────────────────────┐   ┌────────────────────────┐  │     │  │
│  │  │  │ map_builder_node             │   │ localizer_node          │  │     │  │
│  │  │  │ (mapping mode)               │   │ (navigation mode)       │  │     │  │
│  │  │  │  - FAST-LIO2 odometry        │   │  - FAST-LIO2 odometry   │  │     │  │
│  │  │  │  - Loop closure (thread)     │   │  - ICP localization     │  │     │  │
│  │  │  │  - Ground extraction (thread)│   │    (thread)             │  │     │  │
│  │  │  │  - Ground cloud pub (thread) │   │  - Service: reloc/hold/ │  │     │  │
│  │  │  │  - Service: save_map         │   │    start/reloc_check    │  │     │  │
│  │  │  └──────────────────────────────┘   └────────────────────────┘  │     │  │
│  │  │                                         │                         │     │  │
│  │  │                                         ▼                         │     │  │
│  │  │                                  ┌────────────────────────┐      │     │  │
│  │  │                                  │ slam_reloc.py           │      │     │  │
│  │  │                                  │ /initialpose -> /slam_reloc    │     │  │
│  │  │                                  └────────────────────────┘      │     │  │
│  │  └─────────────────────────────────────────────────────────────────┘     │  │
│  │                                    │                                       │  │
│  │                                    ▼                                       │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐     │  │
│  │  │              Perception & Mapping                               │     │  │
│  │  │  ┌───────────────┐  ┌──────────────┐  ┌──────────────┐       │  │  │
│  │  │  │ octomap_server│  │ map_server   │  │ pointcloud_  │       │  │  │
│  │  │  │ /body_cloud   │  │ (2D map)     │  │ to_laserscan │       │  │  │
│  │  │  │ -> 3D octomap │  │ /map topic   │  │ /body_cloud  │       │  │  │
│  │  │  └───────────────┘  └──────────────┘  │ -> /scan     │       │  │  │
│  │  │                                       └──────────────┘       │  │  │
│  │  └─────────────────────────────────────────────────────────────────┘     │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
│                                    │                                            │
│                                    ▼                                            │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                      Hardware Interface                                   │  │
│  │  ┌────────────────────────────────────────────────────────────────────┐  │  │
│  │  │  livox_ros_driver2 (MID360 LiDAR)                                  │  │  │
│  │  │  - /livox/lidar (CustomMsg) - Point cloud data                      │  │  │
│  │  │  - /livox/imu (sensor_msgs/Imu) - IMU data                         │  │  │
│  │  └────────────────────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────────────────┘
```

## Component Responsibilities

| Component | Responsibility | File |
|-----------|----------------|------|
| `map_builder_node` | Mapping mode: FAST-LIO2 + loop closure + ground extraction + map saving | `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` |
| `localizer_node` | Navigation mode: FAST-LIO2 + ICP relocalization against prebuilt map + services | `G1Nav2D/src/fastlio2/src/localizer_node.cpp` |
| `temp_node` | Minimal FAST-LIO2 test node (no mapping/nav features) | `G1Nav2D/src/fastlio2/src/temp_node.cpp` |
| `LIOBuilder` | Core LIO engine: ESIKF filter, ikd-Tree map, IMU preintegration | `G1Nav2D/src/fastlio2/src/lio_builder/lio_builder.cpp` |
| `IMUProcessor` | IMU preintegration, gravity alignment, motion undistortion | `G1Nav2D/src/fastlio2/src/lio_builder/imu_processor.cpp` |
| `IcpLocalizer` | Multi-resolution ICP localization against PCD map | `G1Nav2D/src/fastlio2/src/localizer/icp_localizer.cpp` |
| `LoopClosureThread` | Separate thread: KdTree search + ICP + ISAM2 pose graph optimization | `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` (class inside) |
| `GroundExtractionThread` | Separate thread: z-threshold ground extraction from keyframe clouds | `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` (class inside) |
| `GroundCloudPublishThread` | Separate thread: periodic ground map publishing + auto-save on SIGINT | `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` (class inside) |
| `move_base` | ROS navigation stack: global + local path planning with TEB | Launched from `G1Nav2D/src/movebase/launch/move_base.launch` |
| `costmap_clear` | Clears costmaps when /initialpose is received | `G1Nav2D/src/movebase/src/costmap_clear.cpp` |
| `velocity_smoother_ema` | EMA filter on /cmd_vel, timeout-based stop | `G1Nav2D/src/velocity_smoother_ema/src/velocity_smoother_ema.cpp` |
| `slam_reloc.py` | Bridge RViz 2D Pose Estimate to /slam_reloc service | `G1Nav2D/src/fastlio2/scripts/slam_reloc.py` |
| `YgClient.py` / `CmdVelController` | ROS-to-robot bridge: subscribes /cmd_vel_smooth, sends via DogControllerSDK HTTP | `G1Nav2D/client/YgClient.py` |

## Pattern Overview

**Overall:** ROS1 node-based architecture with threaded auxiliary processing.

**Key Characteristics:**
- **Two-phase pipeline:** Mapping phase (map_builder_node with loop closure) then Navigation phase (localizer_node with pre-built map)
- **Threaded background workers:** Loop closure, ground extraction, and ground publishing each run in dedicated threads sharing data through `SharedData` struct with mutex protection
- **Hybrid localization:** FAST-LIO2 provides real-time state estimation (ESIKF on manifold); ICP-based relocalization corrects drift against a pre-built PCD map
- **Pose graph optimization:** GTSAM ISAM2 incremental solver for loop closure, with multi-iteration refinement on detection
- **Sliding window map:** ikd-Tree incremental kd-tree with `trimMap()` to maintain a localized point cloud map around the robot
- **Dual keyframe system:** `cache_unfiltered_key_poses` (dense, for ground extraction) + `key_poses` (sparse, for loop closure optimization)

## Layers

**Hardware Interface Layer:**
- Purpose: Raw sensor data acquisition
- Location: `G1Nav2D/src/livox_ros_driver2-master/`
- Contains: Livox LiDAR ROS driver, publishes `/livox/lidar` (CustomMsg) and `/livox/imu` (sensor_msgs/Imu)
- Key files: `src/driver_node.cpp`, `src/lds_lidar.cpp`, `src/livox_ros_driver2.cpp`

**SLAM & Localization Layer:**
- Purpose: Real-time state estimation, mapping, and localization
- Location: `G1Nav2D/src/fastlio2/`
- Contains: `LIOBuilder` (core ESIKF engine), `IMUProcessor` (IMU preintegration), `IcpLocalizer` (ICP map matching), `LoopClosureThread` (pose graph optimization), `GroundExtractionThread` (ground plane filtering)
- Depends on: Hardware Interface Layer
- Used by: Navigation Layer (provides `/slam_odom` and TF)

**Perception & Mapping Layer:**
- Purpose: 3D octomap, 2D grid map, laser scan generation
- Location: System-level ROS packages (octomap_server, map_server, pointcloud_to_laserscan)
- Contains: OctoMap 3D occupancy mapping, 2D map server for navigation, point cloud to laser scan conversion
- Depends on: SLAM Layer (receives `/body_cloud`)

**Navigation Layer:**
- Purpose: Path planning and obstacle avoidance
- Location: `G1Nav2D/src/movebase/`
- Contains: move_base with TEB local planner, costmap configuration, costmap clearing
- Depends on: SLAM Layer (odometry), Perception Layer (costmap, laser scan)
- Key files: `launch/move_base.launch`, `param/teb_local_planner_params.yaml`, `param/costmap_params.yaml`

**Velocity Smoothing Layer:**
- Purpose: Smooth velocity commands with EMA filter and timeout stop
- Location: `G1Nav2D/src/velocity_smoother_ema/`
- Contains: EMA-based velocity smoother, subscribes `/cmd_vel` (raw), publishes `/cmd_vel_smooth` (filtered)
- Key files: `src/velocity_smoother_ema.cpp`, `param/smoother.yaml`

**Client Layer:**
- Purpose: Bridge ROS commands to robot hardware
- Location: `G1Nav2D/client/`
- Contains: Python ROS node subscribing `/cmd_vel_smooth`, DogControllerSDK HTTP client sending commands to Unitree G1
- Key files: `YgClient.py`, `DogControllerSDK.py`, `constants.py`

## Data Flow

### Mapping Mode (roslaunch fastlio mapping.launch)

1. Livox MID360 LiDAR publishes `/livox/lidar` and `/livox/imu` via `livox_ros_driver2`
2. `map_builder_node` subscribes to both topics; `MeasureGroup::syncPackage()` temporally syncs IMU+LiDAR frames
3. `LIOBuilder::mapping()` runs IMU preintegration (`IMUProcessor`), point cloud undistortion, downsampling, then ESIKF state update against ikd-Tree map
4. `MapBuilderROS::addKeyPose()` checks translation/rotation thresholds and adds keyframes when exceeded
5. `LoopClosureThread` (separate thread, ~1Hz) performs KdTree radius search + ICP + ISAM2 pose graph optimization, updating shared offset_rot/offset_pos
6. `GroundExtractionThread` (separate thread) extracts ground points by z-threshold from cached unfiltered keyframes
7. `GroundCloudPublishThread` (separate thread, ~1Hz) publishes ground map as `/ground_cloud`
8. On SIGINT, `signalHandler()` auto-saves full map PCD, ground map PCD, and keyframe poses to disk
9. Octomap_server subscribes to `/body_cloud` and builds 3D occupancy grid

### Navigation Mode (roslaunch fastlio navigation.launch)

1. Livox MID360 LiDAR publishes `/livox/lidar` and `/livox/imu`
2. `localizer_node` subscribes and runs FAST-LIO2 for real-time state estimation
3. Current LIO state (`local_rot`, `local_pos`) is copied to `SharedData` for the `LocalizerThread`
4. `LocalizerThread` (separate thread, ~1Hz) runs ICP against the pre-loaded PCD map:
   - Uses `IcpLocalizer::align()` for continuous tracking
   - Uses `IcpLocalizer::multi_align_sync()` when triggered via `/slam_reloc` service
   - Updates `shared_data_->offset_rot/offset_pos` to correct drift
5. TF broadcast: `map` -> `local` (ICP offset) -> `body` (LIO state) -> `base_link` (static, roll=180)
6. `/slam_odom` published as `local` frame odometry
7. `/body_cloud` (body-frame undistorted points) is consumed by:
   - `octomap_server` for 3D occupancy grid
   - `pointcloud_to_laserscan` for 2D laser scan (`/scan`)
8. `map_server` publishes pre-built 2D occupancy grid via `/map` topic
9. `move_base` receives `/slam_odom`, `/scan`, `/map`, computes TEB trajectories, publishes `/cmd_vel`
10. `velocity_smoother_ema` (filter node) applies EMA to `/cmd_vel` and publishes `/cmd_vel_smooth`
11. Optional: User sends 2D Nav Goal in RViz -> `/move_base/goal`
12. User sends 2D Pose Estimate in RViz -> `/initialpose` -> `slam_reloc.py` -> `/slam_reloc` service triggers ICP relocalization

### State Management

**`SharedData` struct** is the core shared state mechanism across threads:
- In `map_builder_node.cpp`: Contains `key_poses`, `cloud_history`, `loop_pairs`, `offset_rot/offset_pos`, `key_pose_added` flag. Protected by `std::mutex`.
- In `localizer_node.cpp`: Contains `local_rot/pos`, `offset_rot/pos`, `initial_guess`, `pose_updated` flag, `service_mutex` and `main_mutex`. Service calls and main loop share state through this struct.
- Pattern: Main thread writes sensor-derived state; background thread reads/modifies and writes corrections back.

## Key Abstractions

**`SharedData` struct:**
- Purpose: Thread-safe shared state between ROS main loop and background processing threads
- Files: `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` (lines 130-143), `G1Nav2D/src/fastlio2/src/localizer_node.cpp` (lines 49-68)
- Pattern: Mutex-guarded struct with flags for data availability

**`Pose6D` struct:**
- Purpose: Keyframe pose with local+global representation, offset algebra for loop closure corrections
- Files: `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` (lines 102-128)
- Pattern: Encapsulates `local_rot/pos` (LIO odometry) and `global_rot/pos` (optimized pose after loop closure). Provides `addOffset()` and `getOffset()` methods to convert between coordinate frames.

**`LIOBuilder` class:**
- Purpose: Core LIO engine wrapper managing ESIKF, ikd-Tree, IMU processor
- File: `G1Nav2D/src/fastlio2/include/lio_builder/lio_builder.h`
- Pattern: Facade over the ESIKF filter + ikd-Tree data structure. Provides `mapping()` as the main entry point per measurement cycle.

**`LoopClosureThread` class:**
- Purpose: Self-contained loop closure detection + pose graph optimization running in a dedicated thread
- Files: `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` (lines 160-614)
- Pattern: Functor class (operator()) running in a `std::thread`. Internal pipeline: loopCheck -> addOdomFactor -> addLoopFactor -> smoothAndUpdate.

**`IcpLocalizer` class:**
- Purpose: Multi-resolution ICP alignment against a pre-loaded PCD map
- File: `G1Nav2D/src/fastlio2/include/localizer/icp_localizer.h`
- Pattern: Two-stage ICP (rough then refine) with configurable search parameters (xy/yaw offsets)

**`MeasureGroup` struct:**
- Purpose: Temporally synchronized IMU + LiDAR measurement bundle
- File: `G1Nav2D/src/fastlio2/include/commons.h` (lines 114-123)
- Key method: `syncPackage()` blocks until IMU buffer covers the LiDAR scan duration

## Entry Points

**mapping.launch (map_builder_node):**
- Location: `G1Nav2D/src/fastlio2/launch/mapping.launch`
- Triggers: `roslaunch fastlio mapping.launch`
- Responsibilities: Full SLAM mapping with loop closure, ground extraction, octomap

**navigation.launch (localizer_node + full stack):**
- Location: `G1Nav2D/src/fastlio2/launch/navigation.launch`
- Triggers: `roslaunch fastlio navigation.launch`
- Responsibilities: Localization, navigation, velocity smoothing, robot control

**map_builder_node main():**
- Location: `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` (lines 1363-1398)
- Responsibilities: Creates SharedData, starts GroundExtractionThread, GroundCloudPublishThread, MapBuilderROS, signal handler for auto-save

**localizer_node main():**
- Location: `G1Nav2D/src/fastlio2/src/localizer_node.cpp` (lines 564-573)
- Responsibilities: Creates SharedData, LocalizerROS, starts LocalizerThread

**YgClient.py main:**
- Location: `G1Nav2D/client/YgClient.py`
- Responsibilities: ROS node subscribing `/cmd_vel_smooth`, sending to DogControllerSDK via HTTP

**velocity_smoother_ema main():**
- Location: `G1Nav2D/src/velocity_smoother_ema/src/velocity_smoother_ema.cpp` (lines 73-84)
- Responsibilities: EMA filter on velocity commands with stop timeout

## ROS Communication Map

**Published Topics:**

| Topic | Type | Publisher Node | Consumers |
|-------|------|----------------|-----------|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | `livox_ros_driver2` | localizer_node, map_builder_node |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_ros_driver2` | localizer_node, map_builder_node |
| `/slam_odom` | `nav_msgs/Odometry` | localizer_node, map_builder_node | move_base (odom_topic) |
| `/body_cloud` | `sensor_msgs/PointCloud2` | localizer_node, map_builder_node | octomap_server, pointcloud_to_laserscan |
| `/local_cloud` | `sensor_msgs/PointCloud2` | localizer_node, map_builder_node | RViz visualization |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | localizer_node | RViz (original point cloud) |
| `/cmd_vel` | `geometry_msgs/Twist` | move_base (TEB) | velocity_smoother_ema |
| `/cmd_vel_smooth` | `geometry_msgs/Twist` | velocity_smoother_ema | YgClient.py |
| `/map` | `nav_msgs/OccupancyGrid` | map_server | move_base costmap |
| `/scan` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan | move_base costmap |
| `/ground_cloud` | `sensor_msgs/PointCloud2` | map_builder_node | RViz visualization |

**Services:**

| Service | Provider | Purpose |
|---------|----------|---------|
| `/slam_reloc` | localizer_node | Trigger ICP relocalization with initial guess |
| `/slam_hold` | localizer_node | Pause localization |
| `/slam_start` | localizer_node | Resume localization |
| `/slam_reloc_check` | localizer_node | Check if relocalization succeeded |
| `/map_convert` | localizer_node | Downsample and convert PCD map (add normals) |
| `/save_map` | map_builder_node | Save current map to PCD file |
| `/move_base/clear_costmaps` | move_base | Clear navigation costmaps |

**TF Tree:**
```
map (global_frame) ──offset──> local (local_frame) ──LIO state──> body
    │                                                               │
    └── static_transform (roll=180°) ───────────────────────────────┘
                                                                     │
                                                                     ▼
                                                                base_link
```

## Architectural Constraints

- **Threading:** Event loop pattern with `ros::Rate::sleep()` + `ros::spinOnce()`. Background threads (loop closure, localization, ground extraction, ground publishing) use the same pattern. No async callbacks; ROS subscribers use callbacks that push to thread-safe deques.
- **Global state:** `terminate_flag` (global bool for graceful shutdown), `g_ground_pub_thread` (global pointer for auto-save on SIGINT), `g_map_path` and similar (default save paths). Located in `map_builder_node.cpp` lines 38, 1342-1345.
- **Circular imports:** Not detected. ROS header-only communications prevent circular dependencies.
- **Data synchronization:** `MeasureGroup::syncPackage()` blocks until IMU data spans the full LiDAR scan time window. This imposes the main loop rate to be bounded by sensor data arrival.

## Anti-Patterns

### Global Variables for State

**What happens:** `map_builder_node.cpp` uses file-scope globals (`terminate_flag`, `g_ground_pub_thread`, `g_map_path`, etc.) for shutdown handling and auto-save.
**Why it's wrong:** Global state makes testing impossible and creates hidden coupling between the signal handler and the ROS node lifecycle.
**Do this instead:** Encapsulate shutdown state within `MapBuilderROS` or use a dedicated `ShutdownManager` singleton. File: `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` lines 38, 1342-1361.

### Monolithic Node with Nested Classes

**What happens:** `map_builder_node.cpp` (1398 lines) contains 6 top-level structs/classes (`ZaxisPriorFactor`, `LoopPair`, `Pose6D`, `SharedData`, `LoopParams`, `LoopClosureThread`, `MapBuilderROS`, `GroundExtractionThread`, `GroundCloudPublishThread`) plus `signalHandler` and `main` -- all in one file.
**Why it's wrong:** Severely violates single-responsibility principle. The file mixes data types, state management, ROS pub/sub, service handling, loop closure, ground extraction, ground publishing, and signal handling.
**Do this instead:** Split into separate compilation units per class (e.g., `loop_closure.h/cpp`, `ground_extraction.h/cpp`, `map_builder_ros.h/cpp`). See `G1Nav2D/src/fastlio2/src/lio_builder/` and `G1Nav2D/src/fastlio2/src/localizer/` for the correct pattern already used elsewhere.

### Hardcoded Absolute Paths

**What happens:** Map save paths are hardcoded as `/root/HongTu/pcd` in `map_builder_node.cpp` line 1392 and in README as mapping mode instructions.
**Why it's wrong:** Breaks when running outside the specific Docker container. Forces path changes via source code edits rather than ROS parameters or launch arguments.
**Do this instead:** Use `ros::param` with sensible defaults and document overrides in launch files. File: `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` lines 1392-1394.

### Global NodeHandle with "/" namespace

**What happens:** `main()` in both nodes passes `nh("/")` to use global namespace, meaning all parameters are loaded from `/` namespace.
**Files:** `G1Nav2D/src/fastlio2/src/map_builder_node.cpp` line 1366, `G1Nav2D/src/fastlio2/src/localizer_node.cpp` line 566.
**Impact:** Parameter collisions are possible with other nodes. Parameters must be loaded via `rosparam` at the global level.

## Error Handling

**Strategy:** Minimal. Errors are typically logged via ROS_WARN/ROS_INFO with no recovery path. Key patterns:
- Subscriber count check before publish (`publisher.getNumSubscribers() == 0` returns early)
- ICP matching success check (`icp_->hasConverged()` / `icp_localizer_->isSuccess()`)
- `share_data.valid = false` in ESIKF when no effective points

**Missing:** No exception handling around PCD I/O, no retry logic for service calls, no fallback behavior when localization fails.

## Cross-Cutting Concerns

**Logging:** ROS console (`ROS_INFO`, `ROS_WARN`) via `ros/ros.h`. No structured logging library.
**Validation:** Parameter loading with default fallbacks (`nh.param<T>(key, var, default)`). No input validation beyond range clipping for LiDAR points.
**Authentication:** None (local only, no network-facing authentication).
**Configuration:** YAML files loaded via `<rosparam command="load">` in launch files. Parameters are namespace-prefixed (e.g., `/lio_builder/det_range`). Config files at `G1Nav2D/src/fastlio2/config/mapping.yaml` and `localize.yaml`.

---

*Architecture analysis: 2026-05-06*
