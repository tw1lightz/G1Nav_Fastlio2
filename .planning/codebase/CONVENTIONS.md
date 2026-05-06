# Coding Conventions

**Analysis Date:** 2026-05-06

## Languages

- **C++14** (`-std=c++14`): Primary language for ROS1 nodes, SLAM pipeline, LiDAR processing. All source in `G1Nav2D/src/fastlio2/`.
- **Python 3**: ROS node scripts (`G1Nav2D/src/fastlio2/scripts/slam_reloc.py`), robot SDK client (`G1Nav2D/client/`).

## Naming Patterns

**Files (C++):**
- `snake_case.cpp` / `snake_case.h` for source and headers under `G1Nav2D/src/fastlio2/src/` and `G1Nav2D/src/fastlio2/include/` — e.g., `map_builder_node.cpp`, `icp_localizer.cpp`, `imu_processor.h`.
- Exceptions: Third-party internal headers (`ikd_Tree.h`, `ikd_Tree.cpp`, `esekfom.hpp`) retain original naming.

**Files (Python):**
- `PascalCase.py` for client modules — e.g., `YgClient.py`, `DogControllerSDK.py`.
- `snake_case.py` for utility scripts — e.g., `slam_reloc.py`.

**Classes:**
- `PascalCase` — e.g. `MapBuilderROS`, `LocalizerROS`, `LoopClosureThread`, `GroundExtractionThread`, `LIOBuilder`, `IMUProcessor`, `IcpLocalizer`, `CmdVelController`.
- ROS node wrapper classes use the suffix `ROS` (`MapBuilderROS`, `LocalizerROS`).

**Functions/Methods:**
- `camelCase` — e.g. `initParams()`, `initSubscribers()`, `addKeyPose()`, `publishCloud()`, `getLidar2BaseFromParam()`, `loopCheck()`, `addOdomFactor()`, `smoothAndUpdate()`.
- Low-level utility functions in `commons.h` use `snake_case` — e.g. `esti_plane()`, `sq_dist()`, `process_noise_cov()`, `eigen2Odometry()`, `pcl2msg()`, `rotate2rpy()`.

**Member Variables:**
- `trailing_underscore_` — e.g. `nh_`, `br_`, `imu_sub_`, `current_time_`, `shared_data_`, `lio_builder_`, `loop_params_`.

**Struct/POD Types:**
- `PascalCase` — e.g. `IMU`, `Pose6D`, `LoopPair`, `LoopParams`, `SharedData`, `LocalMap`, `LioParams`.
- Constructor initializer-list style: `LoopPair(int p, int c, float s, ...) : pre_idx(p), cur_idx(c), score(s) {}`.

**Constants and Macros:**
- `UPPER_SNAKE_CASE` for preprocessor defines — e.g. `NUM_MATCH_POINTS`, `SKEW_SYM_MATRX`, `NUM_MAX_POINTS`, `G_m_s2`.

**Namespaces:**
- `lowercase` — `fastlio` for all library code in headers and implementation files.

**ROS Message/SRV Types:**
- Generated messages in `PascalCase` — e.g. `fastlio::SaveMap`, `fastlio::SlamReLoc`.

## Code Style

**Formatting:**
- No automatic formatter detected (no `.clang-format`, `.editorconfig`, or `biome.json`).
- Indentation: 4 spaces (no tabs).
- Opening braces on the same line for functions, classes, structs, namespaces, `if`/`for`/`while`.
- Closing brace on its own line. No `// namespace X` or similar closing comments (except `// namespace fastlio` at end of `commons.cpp`).

**Pattern from `G1Nav2D/src/fastlio2/src/map_builder_node.cpp`:**
```cpp
class MapBuilderROS
{
public:
    MapBuilderROS(tf2_ros::TransformBroadcaster &br, std::shared_ptr<SharedData> share_data) : br_(br)
    {
        shared_data_ = share_data;
        initPatams();
        initSubscribers();
        ...
    }
```

**Consistency note:** There is a typo `initPatams` (should be `initParams`) in `MapBuilderROS`, and `setSharedDate` (should be `setSharedData`) in `LocalizerROS` — these inconsistencies are present but not fixed.

## Import Organization (C++)

**Ordering pattern observed across all `.cpp` files:**

1. Standard library headers: `#include <map>`, `#include <mutex>`, `#include <vector>`, `#include <thread>`
2. ROS headers: `#include <ros/ros.h>`, `#include <nav_msgs/Odometry.h>`
3. PCL headers: `#include <pcl/io/pcd_io.h>`, `#include <pcl/filters/voxel_grid.h>`
4. Eigen headers: `#include <Eigen/Dense>`
5. GTSAM headers: `#include <gtsam/geometry/Rot3.h>`
6. Project headers (quoted): `#include "lio_builder/lio_builder.h"`, `#include "fastlio/SaveMap.h"`

**Path Aliases:**
- Project headers use `#include "fastlio/ServiceName.h"` for generated SRV headers.
- Internal headers use relative paths from `include/`: `#include "lio_builder/lio_builder.h"`, `#include "localizer/icp_localizer.h"`.

## Error Handling

**Pattern: Return boolean + condition check.**

- Used consistently in service callbacks and data processing functions.
- Example from `G1Nav2D/src/fastlio2/src/map_builder_node.cpp`:
```cpp
if (!icp_->hasConverged() || score > loop_params_.loop_icp_thresh)
    return;
```

**Pattern: Status flag + guard clauses.**

- Worker threads check flags before proceeding (`terminate_flag`, `shared_data_->halt_flag`, `shared_data_->localizer_activate`).
- Early returns for empty data, invalid states.

**Pattern: Service callback returns bool + populates response message.**

- Example from `G1Nav2D/src/fastlio2/src/map_builder_node.cpp`:
```cpp
if (cloud->empty())
{
    res.status = false;
    res.message = "Empty cloud!";
    return false;
}
```

**Exception handling (Python only):**
- `try/except` blocks in `YgClient.py` — wraps SDK calls in try/except with logging.

**No exception handling in C++** — no `try`/`catch` blocks in any of the project C++ code.

## Logging

**Framework:** ROS1 rosconsole for C++, `rospy` for Python.

**Patterns (C++):**
- `ROS_INFO("Detected LOOP: %d %d %f", pre_index, cur_index, score);` — informational messages.
- `ROS_WARN("lidar2base param size error, use identity.");` — warnings for recoverable issues.
- `ROS_INFO("After voxel filter: %lu -> %lu points", cloud->size(), voxel_filtered->size());` — debug/progress info.
- `ROS_WARN("IMU processor not initialized yet, using default gravity vector");` — fallback logging.
- `std::cout` used only for signal handler shutdown messages (`"SHUTTING DOWN MAPPING NODE!"`) and timing debug (commented out).

**Patterns (Python):**
- `rospy.loginfo()` — standard info.
- `rospy.logerr()` — error cases.

**No structured logging** (no JSON logs, no log levels beyond ROS defaults).

## Comments

**Language:** Predominantly Chinese for business logic comments, English for inline technical comments and Doxygen annotations.

**When to Comment:**
- Logical sections marked with numbered steps: `// ===【1】更新ISAM2图优化器... ===`
- Doxygen style: `/** @brief ... */` for class methods.
- Section headings with emoji markers: `// ❌ 5. 没有新关键帧添加，跳过`
- C++ `//` line comments, no `/* */` block comments.

**Pattern from `G1Nav2D/src/fastlio2/src/map_builder_node.cpp`:**
```cpp
/**
 * @brief 回环检测主函数（每次关键帧添加后调用）
 * 
 * 实现流程：
 * 1. 从历史关键帧构建 KD-Tree；
 * 2. 基于当前帧位置查找附近的历史帧；
 * 3. ...
 */
void loopCheck()
```

**Commented-out code:** Present in several files (`localizer_node.cpp` lines 242-268 have a commented-out `relocCallback`, `map_builder_node.cpp` has commented-out timing code and publish calls).

## Function Design

**Size:** Functions range from ~10 lines (simple getters/setters) to ~250 lines (`addKeyPose` in `map_builder_node.cpp`, `relocCallback` in `localizer_node.cpp`). Large functions are common — they handle complete processing pipelines within a single method.

**Parameters:** Pass by `const &` for large objects (Eigen matrices, ROS messages), by value for primitives, by shared_ptr for shared resources.

**Return Values:** Return by value for small types (bool, int), return `void` and modify shared state for processing functions, return `PointCloudXYZI::Ptr` for point cloud operations.

## Module Design

**Exports:**
- Library code lives in `namespace fastlio` within header files under `G1Nav2D/src/fastlio2/include/`.
- ROS node classes (`MapBuilderROS`, `LocalizerROS`) defined entirely in `.cpp` files — not exposed via headers.
- Callback functions, `SharedData` structs, signal handlers are file-scoped or in unnamed namespaces.

**Pattern in ROS node files:**
```cpp
// At top of file
bool terminate_flag = false;

// Free function (e.g., getLidar2BaseFromParam)
Eigen::Matrix4f getLidar2BaseFromParam(const ros::NodeHandle& nh) { ... }

// Struct definitions
struct SharedData { ... };
struct LocalParams { ... };

// Thread worker class
class LoopClosureThread {
public:
    void operator()() { ... }
};

// ROS wrapper class
class MapBuilderROS { ... };

// main()
int main(int argc, char **argv) { ... }
```

**Barrel Files:**
- `commons.h` serves as a barrel for shared types (`IMU`, `LivoxData`, `MeasureGroup`, etc.) and namespace `fastlio` utilities. This file is included by virtually all other headers.

**No separate `.hpp` files** — all main project headers use `.h` extension. (Exception: third-party IKFoM submodule uses `.hpp`.)

## Threading Conventions

**Pattern:** Thread workers are implemented as functor classes with `operator()()`, spawned via `std::thread(std::ref(instance))`.

**Example from `G1Nav2D/src/fastlio2/src/map_builder_node.cpp`:**
```cpp
loop_thread_ = std::make_shared<std::thread>(std::ref(loop_closure_));
```

**Mutex usage:** `std::mutex` with `std::lock_guard` for scoped locking. Manual `lock()`/`unlock()` also present (e.g., in `saveGroundMap`, `GroundExtractionThread`).

**Signal handling:** Global `terminate_flag` set by signal handler (`signalHandler`), checked in main loops and thread workers.

## ROS Conventions

**Package structure:** Standard catkin layout:
```
package.xml
CMakeLists.txt
include/      -- headers
src/          -- .cpp files
launch/       -- .launch files
config/       -- .yaml parameter files
srv/          -- .srv service definitions
scripts/      -- Python executables
rviz/         -- .rviz visualization configs
```

**Topic naming:** Lowercase with underscores: `/slam_odom`, `/body_cloud`, `/local_cloud`, `/ground_cloud`, `/loop_mark`.

**Parameter loading:** `rosparam command="load"` loads YAML files from config/. Parameters accessed via `nh_.param<T>(key, default)`.

**Frame conventions:** `map` (global), `local` (odometry), `body` (sensor/LiDAR frame), `base_link` (robot base).

---

*Convention analysis: 2026-05-06*
