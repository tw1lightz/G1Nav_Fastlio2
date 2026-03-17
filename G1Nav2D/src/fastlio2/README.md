# FASTLIO_SAM_LC

## 主要工作
1. 对原始FASTLIO进行代码重构 
2. 建图线程添加GTSAM做回环
3. 添加定位线程用于基于已知地图的重定位
4. 添加首帧重力对齐(用于传感器非水平放置情况)
5. 目前暂时支持MID_360的传感器(买不起其他传感器)
6. 代码结构更加简单，流程逻辑也更加清晰(非常自以为是的评价)

## 环境说明
```text
ubuntu20.04
ros noetic
pcl 1.10
```

## 编译依赖
1. livox_ros_driver2
2. gtsam (noetic版本 可以使用 sudo apt install ros-noetic-gtsam 直接安装，其他版本可能需要独立安装)
3. pcl

## 启动脚本
1. 建图线程
```shell
roslaunch fastlio mapping.launch
```
2. 定位线程
```shell
roslaunch fastlio localize.launch
```

## 使用自建地图测试导航（推荐：外部PC运行RViz）

假设你已经通过建图得到了：

- 3D 点云地图：`/root/HongTu/test2.pcd`（用于重定位）
- 2D 地图 YAML：`/root/HongTu/map/test2.yaml`（用于 map_server / move_base）

### 1）机器人端启动（不启动RViz）

在机器人（ROS Master）上执行：

```shell
cd ~/HongTu/G1Nav2D
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

roslaunch fastlio navigation.launch
```
说明：

- `pcd_map` 会传给 `slam_reloc.py`，用于调用 `/slam_reloc` 做重定位。
- `map2d_yaml` 会传给 `map_server`，发布 2D 地图话题 `/map_2d`。

### 2）外部PC配置ROS网络并验证话题

外部PC需要能访问机器人IP（示例：机器人 `192.168.123.164`）。

外部PC（Ubuntu）环境变量示例：

```shell
export ROS_MASTER_URI=http://192.168.100.29:11311
export ROS_IP=<外部PC自己的IP>
```

验证能看到话题：

```shell
rostopic list
```

如果看不到话题，优先检查：

- 外部PC与机器人是否同网段、能否 `ping` 通
- `ROS_MASTER_URI` 是否指向机器人
- `ROS_IP` 是否填了外部PC的可达IP（不要填 127.0.0.1）
- 防火墙是否阻断了 11311/随机端口（ROS1会用到随机端口传输话题）

### 3）外部PC启动RViz与必选显示项（Displays）

在外部PC执行：

```shell
rviz
```

RViz 顶部：

- `Fixed Frame` 建议设为 `map`（如果 TF 未连通，可临时设为 `local` 排查）

建议添加的 Displays（插件/显示项）：

- `TF`：检查 `map → local → body → base_link` 等坐标系是否连通
- `Map`：Topic 选 `/map_2d`
- `LaserScan`：Topic 选 `/scan`
- `Odometry`：Topic 选 `/slam_odom`
- （可选）`Path`：Topic 选 `/global_path`、`/local_path`
- （可选）`PointCloud2`：Topic 可选 `/local_cloud`、`/body_cloud`、`/velodyne_points`

### 4）在RViz中进行定位与下发导航目标

工具栏操作：

1. `2D Pose Estimate`：在地图上点击设置初始位姿（发布 `/initialpose`），触发 `slam_reloc.py` 调用 `/slam_reloc`。
2. `2D Nav Goal`：在地图上点击设置目标点（发布到 move_base），开始规划并输出速度。

### 5）常见问题快速排查

- 能看到 `/map_2d` 但看不到 `/scan`：检查点云到激光的节点是否在跑，以及是否有 TF 到 `base_link`。
- RViz 报 TF 错误：先打开 `TF` display，确认 `Fixed Frame` 与 TF 树一致。
- 规划抖动/误判障碍：通常与地面点/外参/高度阈值有关（点云→scan 的 `min_height/max_height` 也会影响）。

### 6）2D 地图与 3D PCD 对不上的典型原因与解决

如果你在 RViz 里看到：`/local_cloud` 明显对不上 `/map_2d`（边界/尺度/方向完全不一致），通常不是 TF “小偏差”，而是 **2D 地图和 3D PCD 根本不是同一套坐标系/同一次建图产物**。

推荐做法：在同一次建图过程中，同时保存 PCD 和 2D OccupancyGrid（从 `octomap_server` 的 `projected_map` 保存）。示例：

1）启动建图（包含 `octomap_server`）：

```shell
roslaunch fastlio mapping.launch
```

2）保存 3D 点云地图（PCD）：

```shell
rosservice call /save_map "{save_path: '/root/HongTu/test2.pcd', resolution: 0.0}"
```

3）保存 2D 地图（pgm + yaml）：

```shell
mkdir -p /root/HongTu/map
rosrun map_server map_saver -f /root/HongTu/map/test2 map:=/projected_map
```

保存完成后，用 `navigation.launch` 的默认参数即可：

- `pcd_map=/root/HongTu/test2.pcd`
- `map2d_yaml=/root/HongTu/map/test2.yaml`

提示：若 `2D Pose Estimate` 看起来“没反应”，请优先用 `/slam_reloc_check` 确认重定位是否真正成功。

## 服务脚本
1. 保存地图
```shell
rosservice call /save_map "{save_path: '/home/nvidia/map_delet.pcd', resolution: 0.0}"


```
**目前resolution没用(需要降采样，可离线自行降采样)**

2. 重定位
```shell
rosservice call /slam_reloc "{pcd_path: 'you_pcd_path.pcd', x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}" 
```

## 点云距离裁剪（可用于抑制玻璃远点）
玻璃/反射等场景可能产生非常远的离群点，影响建图质量。这里提供在 FastLIO 输入前对点云做半径裁剪（单位：米）的参数：

- `lio_builder/min_point_range`：最小距离，0 表示禁用
- `lio_builder/max_point_range`：最大距离，0 表示禁用

可在建图与定位的 YAML 中配置：
- `config/mapping.yaml`
- `config/localize.yaml`

示例（室内可先从 20~30m 试起）：
```yaml
lio_builder:
  min_point_range: 0.0
  max_point_range: 25.0
```

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [FASTLIO-SAM](https://github.com/kahowang/FAST_LIO_SAM)
3. [FASTLIO-LC](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)