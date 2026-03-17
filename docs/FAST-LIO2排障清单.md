# FAST-LIO2 排障清单（G1 + MID360）

## 前置条件
```bash
# 确保在容器内/已 source ROS 环境
docker exec -it ros1_noetic_hongtu /bin/bash
cd /root/HongTu/G1Nav2D
source devel/setup.bash
```

## 第一步：启动驱动（不启动 FAST-LIO）
```bash
# 只启动 Livox 驱动，用于验证话题和数据
roslaunch livox_ros_driver2 msg_MID360.launch
```

## 检查项 1：重力方向/点云是否倒置

### 1.1 检查 IMU 数据（静止状态）
```bash
# 新终端，观察 IMU 线加速度
rostopic echo /livox/imu

# 关注 linear_acceleration.z 字段：
# - 本仓库常见驱动输出接近 ±1.0（约等于 ±1g；不是 ±9.81 m/s²）
# - 符号取决于你的坐标系定义（通常 z 轴向上时为正）
# - 如果是 -1.0 且你定义 z 轴向上，说明坐标系反了
```

**判断标准**：
- ✅ 正常：`linear_acceleration.z` 接近 +1.0（或 -1.0，取决于坐标系）
- ❌ 异常：值为 0、值远离 1.0、或符号与预期相反
- ⚠️ **倒装但"建图正常"**：如果 z ≈ -1.0（倒装）但建图看起来没问题，通常说明系统内部坐标系仍然自洽（或你在其它环节做了补偿）；**仍建议显式配置驱动 roll=180°**，避免后续 TF/导航/重定位阶段出现不一致。

**为什么倒装但建图看起来正常？（这仓库的常见原因）**
- 如果 LiDAR 与 IMU 在同一倒装坐标系下输出（都用 livox_frame），LIO 仍可能正常工作：它只要求两者的相对关系自洽。
- 本仓库 fastlio2 会做首帧重力对齐（mapping/localize.yaml 里 `lio_builder/align_gravity: true`），能一定程度容忍坐标系方向差异。
- 真正容易出问题的是 **TF 链路与导航期望的 base_link/map**：即使建图看起来正常，到了 `/scan`、`/map_2d`、重定位/导航阶段可能暴露。

**解决方案**（倒装时）：
修改 `G1Nav2D/src/livox_ros_driver2-master/config/MID360_config.json`：
```json
"MID360": {
  "lidar_net_info": { ... },
  "host_net_info": { ... },
  "lidar_configs": [
    {
      "extrinsic_parameter": {
        "roll": 180.0,    // G1 倒装时设为 180
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```
配置后**建议同时修改** `mapping.yaml`：
```yaml
# 本仓库外参在 lio_builder/imu_ext_rot 与 imu_ext_pos（IMU↔LiDAR）
# 如果你已经确定外参，直接把这两个参数填准并固定
lio_builder:
  imu_ext_rot: [ ... 9 values ... ]
  imu_ext_pos: [x, y, z]
```
修改后重新编译驱动：
```bash
cd /root/HongTu/G1Nav2D/src/livox_ros_driver2-master
./build.sh ROS1
cd /root/HongTu/G1Nav2D
catkin_make
```

### 1.2 检查点云朝向（可选）
```bash
# 在外部 PC 上启动 RViz（需配置 ROS_MASTER_URI）
export ROS_MASTER_URI=http://192.168.123.164:11311
export ROS_IP=<你的外部PC IP>
rviz

# 添加 PointCloud2 显示，topic 选 /livox/lidar
# Fixed Frame 设为 livox_frame
# 观察点云是否"倒立"
```

---

## 检查项 2：点云是否包含逐点时间戳

```bash
# 查看点云消息结构
rostopic echo /livox/lidar -n1

# 或用 rosmsg 检查字段
rostopic type /livox/lidar | rosmsg show

# 应该能看到 fields 里有 'time' 或 'timestamp' 字段
# 如果没有，FAST-LIO2 会报错："Failed to find match for field 'time'"
```

**判断标准**：
- ✅ 正常：`fields` 包含 `name: "time"` 或类似时间戳字段
- ❌ 异常：只有 `x, y, z, intensity`，缺少时间字段

**解决方案**（异常时）：
检查 `livox_ros_driver2` 配置中的消息类型，确保使用带时间戳的点云格式（通常 MID360 驱动默认会加）。

---

## 检查项 3：时间同步配置

### 3.1 检查当前配置
```bash
# 本仓库 fastlio2 没有暴露 time_offset_lidar_to_imu 这类 YAML 参数。
# 它在代码里用 IMU/LiDAR 的 header.stamp + 点内 offset_time 做同步（MeasureGroup::syncPackage）。
# 所以时间不同步通常要从：驱动时间戳、系统时钟、ROS 时间来源（/use_sim_time）去修。
```

### 3.2 验证话题时间戳（运行时）
```bash
# 同时查看 LiDAR 和 IMU 的时间戳
rostopic echo /livox/lidar/header/stamp &
rostopic echo /livox/imu/header/stamp &

# 两个时间戳应该接近（差异在毫秒级）
# 如果差异很大（>100ms），同步模块会等不到数据，表现为：建图抖动/卡住/不同步
```

---

## 检查项 4：外参配置（本仓库：imu_ext_rot / imu_ext_pos）

### 4.1 检查当前外参配置
```bash
cat /root/HongTu/G1Nav2D/src/fastlio2/config/mapping.yaml | grep -A 5 imu_ext
```

应该看到类似：
```yaml
lio_builder:
  imu_ext_rot: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # 旋转矩阵（行优先，9 个数）
  imu_ext_pos: [x, y, z]                     # 平移（米）
```

**判断标准**：
- ✅ 已知外参：把 `lio_builder/imu_ext_rot` 与 `imu_ext_pos` 填准并固定
- ⚠️ 未知外参：优先用 LI-Init 做一次外参标定后回填；不建议在这份仓库里依赖"在线估计开关"（YAML 里本来也没有）

### 4.2 验证外参是否合理（运行 mapping 时）
```bash
# 启动建图
roslaunch fastlio mapping.launch

# 观察效果：轨迹是否抖动、地图是否扭曲、静止时是否漂移。
# 如果外参不对，常见表现是：地图扭曲/平面倾斜/重力对齐失败。
```

**激励建议**（外参在线估计时）：
1. 开始时静止 5-10 秒（让系统初始化）
2. 缓慢转动（yaw/pitch/roll 都转一转）
3. 平移 + 加减速（让系统"看出来"LiDAR 和 IMU 的相对位置）
4. 持续运动至少 30 秒以上

**如果想用 LI-Init 离线标定**（推荐）：
参考：https://github.com/hku-mars/LiDAR_IMU_Init
```bash
# 录制 rosbag（包含 /livox/imu 和 /livox/lidar）
rosbag record /livox/imu /livox/lidar -O calibration.bag

# 按 LI-Init README 运行标定，得到外参后填回 mapping.yaml
# 并设置 extrinsic_est_en: false
```

---

## 检查项 5：运行建图，观察实际效果

```bash
# 启动完整建图流程
roslaunch fastlio mapping.launch

# 在外部 PC 上启动 RViz
export ROS_MASTER_URI=http://192.168.123.164:11311
export ROS_IP=<你的外部PC IP>
rviz -d /root/HongTu/G1Nav2D/src/fastlio2/rviz/mapping.rviz
```

**观察点与常见问题**：

| 现象 | 可能原因 | 排查方向 |
|------|----------|----------|
| 点云严重抖动/跳跃 | 时间同步问题 | 检查 IMU/LiDAR header.stamp 差异、系统时钟/ROS 时间来源（/use_sim_time） |
| 地图"飘移"/"扭曲" | 外参不准 | 重新标定外参或增加激励 |
| 静止时里程计漂移 | IMU bias/噪声参数不对 | 调整 `b_acc_cov`、`b_gyr_cov` |
| 点云"倒立" | 坐标系配置错误 | 检查驱动的 `extrinsic_parameter` |
| 报错 "Failed to find match for field 'time'" | 点云缺时间戳 | 检查驱动配置/点云消息类型 |
| IMU 数据全是 0 或 NaN | 驱动未正常启动 | 检查 Livox 网络连接、IP 配置 |

---

## 快速排障流程总结

```
1. 启动驱动 → rostopic echo /livox/imu 
  → 检查 linear_acceleration.z 是否接近 ±1.0（约等于 ±1g）
   
2. rostopic echo /livox/lidar -n1 
   → 检查 fields 是否包含 'time' 字段
   
3. 查看配置文件 mapping.yaml 
  → lio_builder/align_gravity=true；imu_ext_rot/imu_ext_pos 是否合理
   
4. roslaunch fastlio mapping.launch 
   → 观察终端输出和 RViz 中的效果
   
5. 如果有问题 → 按上述"观察点与常见问题"表格定位根因
```

---

## 附：常用调试命令速查

```bash
# 查看所有话题
rostopic list

# 查看话题信息
rostopic info /livox/imu
rostopic info /livox/lidar

# 查看话题频率
rostopic hz /livox/imu
rostopic hz /livox/lidar

# 查看 TF 树
rosrun tf view_frames
evince frames.pdf  # 或 xdg-open frames.pdf

# 查看当前参数
rosparam get /

# 实时监控某个参数
rosparam get /lio_builder/imu_ext_rot
rosparam get /lio_builder/imu_ext_pos
```

---

## 参考文档
- FAST-LIO2 官方 README：/root/HongTu/G1Nav2D/src/fastlio2/README.md
- Livox 驱动配置：/root/HongTu/G1Nav2D/src/livox_ros_driver2-master/README.md
- LI-Init 外参标定：https://github.com/hku-mars/LiDAR_IMU_Init
