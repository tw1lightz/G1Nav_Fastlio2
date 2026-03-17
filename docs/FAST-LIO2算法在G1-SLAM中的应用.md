# FAST-LIO2 算法在 G1-SLAM 项目中的应用

> **Tags**: `FAST-LIO2` `SLAM` `LiDAR-Inertial` `ikd-Tree` `ESIKF` `IKFoM` `ROS1-Noetic` `Livox-MID360` `点云配准` `IMU预积分` `回环检测` `GTSAM` `导航` `重定位`

---

## 一、FAST-LIO2 核心原理

### 1.1 算法定位

FAST-LIO2（Fast LiDAR-Inertial Odometry 2）是香港大学 MARS 实验室提出的高性能激光惯导里程计算法，论文发表于 IEEE T-RO 2022。相比初代 FAST-LIO，FAST-LIO2 做了两项关键改进：

1. **去掉特征提取**：不再区分边缘/平面特征，直接对原始点云做点-面配准，大幅提升处理速度和对退化场景的鲁棒性。
2. **引入 ikd-Tree**：使用增量式 KD 树维护全局地图，支持高效的增删查操作，替代了初代的静态 KD-Tree。

### 1.2 算法流程

```
IMU 数据 ──→ 前向传播（预积分）──→ 反向传播（点云去畸变）
                                         ↓
LiDAR 数据 ──────────────────→ 体素下采样
                                         ↓
                              ikd-Tree 近邻搜索
                                         ↓
                              点-面残差计算
                                         ↓
                           迭代误差状态卡尔曼滤波
                            (Iterated ESIKF)
                                         ↓
                              状态估计 + 地图增量更新
```

### 1.3 三大核心组件

#### 1.3.1 迭代误差状态卡尔曼滤波（Iterated ESIKF）

基于 IKFoM（Iterated Kalman Filters on Manifolds）工具库实现，对状态空间建模在流形上进行滤波。

**状态向量**（24 维，定义在 `commons.h`）：

```cpp
MTK_BUILD_MANIFOLD(state_ikfom,
    ((vect3, pos))           // 位置 (3)
    ((SO3, rot))             // 旋转 (SO3)
    ((SO3, offset_R_L_I))   // LiDAR→IMU 旋转外参 (SO3)
    ((vect3, offset_T_L_I)) // LiDAR→IMU 平移外参 (3)
    ((vect3, vel))           // 速度 (3)
    ((vect3, bg))            // 陀螺仪偏置 (3)
    ((vect3, ba))            // 加速度计偏置 (3)
    ((S2, grav))             // 重力方向 (S2 流形, 2)
);
```

核心更新调用：
```cpp
kf_->update_iterated_dyn_share_modified(0.001, solve_H_time);
```

#### 1.3.2 ikd-Tree（增量式 KD 树）

- 支持对地图点云的**增量插入**与**按区域批量删除**
- 配合滑动窗口局部地图，实现高效的在线地图维护
- 源码位于 `include/ikd-Tree/ikd_Tree.h` 和 `ikd_Tree.cpp`

关键操作：
```cpp
ikdtree_->Build(point_world->points);         // 初始化建树
ikdtree_->Nearest_Search(point, K, ...);       // K近邻搜索
ikdtree_->Add_Points(point_to_add, true);      // 增量添加（带下采样）
ikdtree_->Delete_Point_Boxes(cub_to_rm);       // 按包围盒批量删除
```

#### 1.3.3 直接点-面配准（无特征提取）

不提取 edge/planar 特征，直接对每个下采样点在 ikd-Tree 中搜索 5 个最近邻 → 拟合平面 → 计算点到面距离作为观测残差：

```cpp
// 近邻搜索
ikdtree_->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, point_sq_dist);

// 平面拟合 + 点面距离
if (esti_plane(pabcd, points_near, 0.1)) {
    double pd2 = pabcd(0)*x + pabcd(1)*y + pabcd(2)*z + pabcd(3);
    double s = 1 - 0.9 * fabs(pd2) / sqrt(point_body_vec.norm());
    if (s > 0.9) { /* 接受该残差 */ }
}
```

### 1.4 IMU 预积分与点云去畸变

`IMUProcessor` 类负责：

1. **静态初始化**：累积前 20 帧 IMU 数据估计初始重力方向和陀螺仪偏置
2. **重力对齐**：利用初始加速度方向计算传感器相对于重力的旋转
3. **前向传播**：在相邻 IMU 帧之间执行 ESIKF 预测步骤
4. **反向传播/去畸变**：利用 IMU 预测的逐帧位姿，将点云中每个点补偿到帧尾时刻，消除运动畸变

---

## 二、在 G1-SLAM 项目中的应用

### 2.1 项目概况

本项目（FASTLIO_SAM_LC）在 FAST-LIO2 基础上进行了代码重构与功能扩展：

| 扩展功能 | 说明 |
|----------|------|
| GTSAM 回环检测 | 建图线程中加入因子图优化，支持 ICP 回环闭合 |
| 基于地图的重定位 | 导航时加载已有 PCD 地图，通过双层 ICP 实现全局重定位 |
| 首帧重力对齐 | 支持传感器非水平安装（如 MID360 倒装）时自动校正 |
| 动态物体过滤 | 保存地图时基于观测次数过滤行人等动态物体 |
| 点云距离裁剪 | 可配置 `min_point_range` / `max_point_range` 限制有效点云范围 |

### 2.2 使用场景

#### 场景一：3D 建图（mapping）

```bash
roslaunch fastlio mapping.launch
```

- 启动 Livox MID360 驱动 + `map_builder_node`
- LIO 实时估计位姿并构建 3D 点云地图
- GTSAM 后端线程持续进行回环检测与图优化
- 支持保存 3D PCD 地图和 2D 栅格地图

#### 场景二：导航定位（navigation）

```bash
roslaunch fastlio navigation.launch pcd_map:=/root/HongTu/test2.pcd map2d_yaml:=/root/HongTu/map/test2.yaml
```

- 启动 `localizer_node`，加载已有 PCD 地图
- LIO 提供实时里程计（`/slam_odom`）
- 独立重定位线程通过双层 ICP 持续校正全局位姿
- 下游 move_base 使用校正后的位姿进行路径规划

### 2.3 关键配置文件与参数解析

#### 2.3.1 建图配置：`fastlio2/config/mapping.yaml`

```yaml
# === 坐标系 ===
map_frame: map           # 全局地图坐标系
local_frame: local       # 局部坐标系
body_frame: body         # 机体坐标系

# === 传感器话题 ===
imu_topic: /livox/imu    # IMU 数据话题
livox_topic: /livox/lidar # Livox 点云话题

# === 频率 ===
local_rate: 20.0         # LIO 主循环频率 (Hz)
loop_rate: 1.0           # 回环检测频率 (Hz)

# === LIO 核心参数 ===
lio_builder:
  det_range: 100.0       # 局部地图检测范围 (m)，决定局部地图何时需要平移
  cube_len: 500.0        # 局部地图立方体边长 (m)
  resolution: 0.2        # 体素下采样分辨率 (m)，同时用于 ikd-Tree 下采样
  move_thresh: 1.5       # 触发局部地图平移的阈值系数
  min_point_range: 0.0   # 最小有效点云距离 (m)，0=不限制
  max_point_range: 25.0  # 最大有效点云距离 (m)，过滤远距离噪点
  align_gravity: true    # 是否启用首帧重力对齐（倒装传感器必须开启）
  imu_ext_rot: [1,0,0, 0,1,0, 0,0,1]  # IMU→LiDAR 旋转外参 (行优先 3×3)
  imu_ext_pos: [-0.011, -0.02329, 0.04412]  # IMU→LiDAR 平移外参 (m)

# === GTSAM 回环检测参数 ===
loop_closure:
  activate: true                  # 是否启用回环检测
  rad_thresh: 0.2                 # 关键帧角度差阈值 (rad)
  dist_thresh: 1.0                # 关键帧距离差阈值 (m)
  time_thresh: 30.0               # 回环最小时间间隔 (s)
  loop_pose_search_radius: 2.0    # 回环候选搜索半径 (m)
  loop_pose_index_thresh: 20      # 回环最小关键帧索引差
  submap_resolution: 0.2          # 回环子图分辨率 (m)
  submap_search_num: 25           # 构建子图时前后各取的关键帧数
  loop_icp_thresh: 0.1            # ICP 收敛适应度阈值

# === 地图保存时的动态物体过滤 ===
dynamic_filter:
  enable: true
  voxel_size: 0.1          # 过滤体素大小 (m)
  min_observations: 3      # 最少观测次数，低于此值视为动态物体
  sor_mean_k: 20           # 统计滤波邻居数
  sor_stddev: 1.5          # 统计滤波标准差倍数
```

#### 2.3.2 定位配置：`fastlio2/config/localize.yaml`

```yaml
# === LIO 参数（与 mapping.yaml 相同结构） ===
lio_builder:
  # ... 同上 ...

# === 重定位参数 ===
localizer:
  refine_resolution: 0.2   # 精配准体素分辨率 (m)
  rough_resolution: 0.5    # 粗配准体素分辨率 (m)
  refine_iter: 5           # 精配准 ICP 迭代次数
  rough_iter: 10           # 粗配准 ICP 迭代次数
  thresh: 0.15             # ICP 收敛适应度阈值
  xy_offset: 2.0           # 重定位初始搜索 XY 偏移范围 (m)
  yaw_offset: 1.0          # 重定位初始搜索 Yaw 偏移量
  yaw_resolution: 0.5      # 重定位 Yaw 搜索分辨率 (rad)
```

#### 2.3.3 其他相关配置

| 配置文件 | 作用 |
|----------|------|
| `livox_ros_driver2-master/config/MID360_config.json` | 雷达 IP、外参（`extrinsic_parameter.roll: 180.0` 用于倒装） |
| `movebase/param/` | move_base 代价地图参数（膨胀层、障碍物层等） |
| `pointcloud_to_laserscan/launch/point_to_scan.launch` | 3D→2D 激光扫描转换（`min_height` / `max_height`） |

### 2.4 数据流

#### 2.4.1 建图模式数据流

```
                    Livox MID360
                    ┌──────────┐
                    │ /livox/imu │────────────┐
                    │ /livox/lidar│───────┐    │
                    └──────────┘       │    │
                                       ▼    ▼
                              ┌─────────────────┐
                              │ map_builder_node │
                              │                 │
                              │  MeasureGroup    │ ← 时间戳同步 IMU + LiDAR
                              │       ↓         │
                              │  IMUProcessor    │ ← 前向传播 + 点云去畸变
                              │       ↓         │
                              │  LIOBuilder      │ ← 体素下采样 → ESIKF 更新 → ikd-Tree 增量更新
                              │       ↓         │
                              │  LoopClosure     │ ← GTSAM 因子图 + ICP 回环
                              │       ↓         │
                              │  发布话题         │
                              └────────┬────────┘
                                       │
               ┌───────────────────────┼───────────────────────┐
               ▼                       ▼                       ▼
        /slam_odom              /body_cloud             /map (TF)
     (nav_msgs/Odometry)   (sensor_msgs/PointCloud2)   map → local → body
```

#### 2.4.2 导航模式数据流

```
  Livox MID360                                      已有地图
  ┌──────────┐                                ┌─────────────────┐
  │ /livox/imu │──┐                           │ test2.pcd (3D)  │
  │ /livox/lidar│─┐│                           │ test2.yaml (2D) │
  └──────────┘ ││                           └────────┬────────┘
               ▼▼                                    │
     ┌────────────────┐                              │
     │ localizer_node │                              │
     │                │◄─────── ICP 重定位 ──────────┘
     │  LIOBuilder    │ ← 实时 LIO 里程计
     │  IcpLocalizer  │ ← 粗→精双层 ICP 校正全局位姿
     └───────┬────────┘
             │
             ▼
      /slam_odom ──→ body2any_pointcloud ──→ /base_link_cloud
                                                     │
                                              point_to_scan
                                                     │
                                                  /scan
                                                     │
                      map_server (/map_2d) ──→  move_base ──→ /cmd_vel
```

### 2.5 核心代码位置

#### 2.5.1 目录结构

```
G1Nav2D/src/fastlio2/
├── config/
│   ├── mapping.yaml          # 建图参数配置
│   └── localize.yaml         # 定位参数配置
├── include/
│   ├── commons.h             # 状态流形定义、公共类型、辅助函数
│   ├── ikd-Tree/
│   │   ├── ikd_Tree.h        # 增量式 KD 树实现
│   │   └── ikd_Tree.cpp
│   ├── IKFoM_toolkit/
│   │   ├── esekfom/
│   │   │   └── esekfom.hpp   # 迭代 ESIKF 实现（IKFoM 库核心）
│   │   └── mtk/              # 流形工具包（SO3, S2, vect 等类型）
│   ├── lio_builder/
│   │   ├── lio_builder.h     # LIO 核心类声明（状态机、ikd-Tree、ESIKF）
│   │   └── imu_processor.h   # IMU 预积分与点云去畸变
│   └── localizer/
│       └── icp_localizer.h   # 双层 ICP 重定位器
├── launch/
│   ├── mapping.launch        # 建图入口
│   ├── navigation.launch     # 导航入口（含重定位、move_base、点云转换）
│   ├── gridmap_load.launch   # 2D 栅格地图加载
│   └── octomap_server.launch # OctoMap 服务
├── scripts/
│   └── slam_reloc.py         # 重定位服务节点（Python）
└── src/
    ├── map_builder_node.cpp   # 建图主节点（含 GTSAM 回环线程）
    ├── localizer_node.cpp     # 定位主节点（含 ICP 重定位线程）
    ├── commons.cpp            # 公共函数实现
    └── lio_builder/
        ├── lio_builder.cpp    # LIO 核心逻辑：mapping / trimMap / increaseMap / sharedUpdateFunc
        └── imu_processor.cpp  # IMU 初始化、预积分、去畸变实现
```

#### 2.5.2 关键代码索引

| 功能 | 文件 | 关键函数/位置 |
|------|------|--------------|
| 状态流形定义 | `include/commons.h` | `MTK_BUILD_MANIFOLD(state_ikfom, ...)` |
| ESIKF 实现 | `include/IKFoM_toolkit/esekfom/esekfom.hpp` | `esekf::predict()`, `update_iterated_dyn_share_modified()` |
| LIO 主循环 | `src/lio_builder/lio_builder.cpp` | `LIOBuilder::mapping()` |
| 点-面残差计算 | `src/lio_builder/lio_builder.cpp` | `LIOBuilder::sharedUpdateFunc()` |
| 局部地图滑窗 | `src/lio_builder/lio_builder.cpp` | `LIOBuilder::trimMap()` |
| 地图增量更新 | `src/lio_builder/lio_builder.cpp` | `LIOBuilder::increaseMap()` |
| IMU 初始化 | `src/lio_builder/imu_processor.cpp` | `IMUProcessor::init()` |
| 点云去畸变 | `src/lio_builder/imu_processor.cpp` | `IMUProcessor::undistortPointcloud()` |
| ikd-Tree 操作 | `include/ikd-Tree/ikd_Tree.h` | `Build()`, `Nearest_Search()`, `Add_Points()`, `Delete_Point_Boxes()` |
| GTSAM 回环 | `src/map_builder_node.cpp` | `LoopClosureThread::operator()()` |
| 双层 ICP 重定位 | `src/localizer/icp_localizer.cpp` | `IcpLocalizer::align()`, `multi_align_sync()` |
| 重力对齐逆变换 | `src/localizer_node.cpp` | `LocalizerROS::relocCallback()` |

### 2.6 Launch 文件解析

#### 建图: `mapping.launch`

```xml
<!-- 1. 启动 Livox MID360 驱动，发布 /livox/imu 和 /livox/lidar -->
<include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360.launch"/>
<!-- 2. 加载建图参数到参数服务器 -->
<rosparam command="load" file="$(find fastlio)/config/mapping.yaml" />
<!-- 3. 启动建图节点 -->
<node pkg="fastlio" type="map_builder_node" name="map_builder_node" output="screen"/>
<!-- 4. 启动 OctoMap 服务（3D→2D 投影地图） -->
<include file="$(find fastlio)/launch/octomap_server.launch"/>
```

#### 导航: `navigation.launch`

```xml
<!-- 1. TF: body → base_link（传感器倒装180°补偿） -->
<node pkg="tf" type="static_transform_publisher" name="base_to_body"
      args="0 0 0 0 0 3.1416 /body /base_link 10" />
<!-- 2. Livox 驱动 -->
<include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360.launch"/>
<!-- 3. 加载定位参数 + 启动定位节点 -->
<rosparam command="load" file="$(find fastlio)/config/localize.yaml" />
<node pkg="fastlio" type="localizer_node" name="localizer_node" output="screen"/>
<!-- 4. 重定位辅助脚本 -->
<node pkg="fastlio" type="slam_reloc.py" name="slam_reloc" output="screen">
    <param name="pcd_path" value="$(arg pcd_map)" />
</node>
<!-- 5. 2D 地图服务 -->
<include file="$(find fastlio)/launch/gridmap_load.launch">
    <arg name="2dmap_file" value="$(arg map2d_yaml)"/>
</include>
<!-- 6. move_base 路径规划 -->
<include file="$(find xju_pnc)/launch/move_base.launch" />
<!-- 7. 3D点云 → 2D激光扫描 -->
<include file="$(find pointcloud_to_laserscan)/launch/point_to_scan.launch" />
<!-- 8. body坐标系点云转换到 base_link -->
<include file="$(find tool)/launch/body2any_pointcloud.launch"/>
<!-- 9. 速度平滑 -->
<include file="$(find velocity_smoother_ema)/launch/velocity_smoother_ema.launch"/>
```

### 2.7 TF 坐标系链路

```
map ──(localizer校正)──→ local ──(LIO里程计)──→ body ──(static 180°)──→ base_link
```

- `map → local`：由重定位线程（IcpLocalizer）发布，表示全局校正偏移
- `local → body`：由 LIO 实时发布，表示局部里程计位姿
- `body → base_link`：静态 TF，补偿 MID360 倒装的 180° 翻转

---

## 三、与原版 FAST-LIO2 的差异总结

| 方面 | 原版 FAST-LIO2 | 本项目（FASTLIO_SAM_LC） |
|------|----------------|-------------------------|
| 后端优化 | 无 | GTSAM ISAM2 + ICP 回环检测 |
| 重定位 | 无 | 双层 ICP（粗+精）全局重定位 |
| 重力对齐 | 无 | 支持非水平安装的首帧重力对齐 |
| 动态过滤 | 无 | 基于观测次数的体素级动态物体过滤 |
| 点云裁剪 | 无 | 可配置 `min/max_point_range` |
| 代码结构 | 单文件为主 | 模块化重构（LIOBuilder / IMUProcessor / IcpLocalizer） |
| 传感器 | 多种 LiDAR | 目前仅适配 Livox MID360 |
