# HongTu：AI Coding Agent 指南

## 仓库总览
宇树G1人形机器人软件集合，三大模块：
| 模块 | 路径 | 技术栈 |
|------|------|--------|
| ROS1 导航/SLAM | `G1Nav2D/` | Noetic, FAST-LIO2, move_base |
| 语音交互 + MCP | `PythonProject/py-xiaozhi-main/` | Python 3.8+, asyncio |
| 运动控制 SDK | `unitree_sdk2_python/` | CycloneDDS, Python |

## 架构关键点

### ROS 话题管线（改一处须检查全链路）
```
/livox/lidar  body2any_pointcloud  /base_link_cloud  point_to_scan  /scan
                                                                           
/livox/imu  localizer_node  /slam_odom          move_base  /map_2d  map_server
```
**核心 launch 文件**：
- 导航主入口：`fastlio2/launch/navigation.launch`（组合 localize + map_server + move_base）
- 点云转换：`tool/launch/body2any_pointcloud.launch`（frame 映射）
- 激光生成：`pointcloud_to_laserscan/launch/point_to_scan.launch`（`min_height/max_height` 参数）
- 2D 地图：`fastlio2/launch/gridmap_load.launch`（topic `/map_2d`）

### 参数配置位置
| 参数类型 | 文件 |
|----------|------|
| SLAM/重定位 | `fastlio2/config/mapping.yaml`, `localize.yaml` |
| 点云裁剪 | `lio_builder/min_point_range`, `max_point_range` |
| 雷达 IP/外参 | `livox_ros_driver2-master/config/MID360_config.json` |
| 导航代价地图 | `movebase/param/`（ROS 包名 `xju_pnc`） |

## 常用命令

### 编译（容器内）
```bash
cd /root/HongTu/G1Nav2D/src/livox_ros_driver2-master && ./build.sh ROS1
cd /root/HongTu/G1Nav2D && catkin_make
source devel/setup.bash
```

### 建图与保存
```bash
roslaunch fastlio mapping.launch
# 保存 3D 点云
rosservice call /save_map "{save_path: '/root/HongTu/test2.pcd', resolution: 0.0}"
# 保存 2D 地图（同一建图会话）
rosrun map_server map_saver -f /root/HongTu/map/test2 map:=/projected_map
```

### 导航启动
```bash
roslaunch fastlio navigation.launch pcd_map:=/root/HongTu/test2.pcd map2d_yaml:=/root/HongTu/map/test2.yaml
```

## 硬编码路径（ 部署时必须修改）
仓库中存在大量 `/home/zhuo/...` 路径，部署到 G1 时需改为 `/root/HongTu/...`：
- `py-xiaozhi-main/src/mcp/tools/*/tools.py` 的 `python_path`, `script_path`
- `PythonProject/daohang/*.py` 的脚本路径
- `PythonProject/point_nav/point*.py` 的音频文件路径

## MCP 工具扩展模式
添加新语音命令触发的动作：
1. 在 `py-xiaozhi-main/src/mcp/tools/` 下创建新目录
2. 实现 `tools.py`，导出 `async def my_function(args: dict) -> str`
3. 在 `mcp_server.py` 的 `add_common_tools()` 中注册

### 现有 MCP 工具
| 工具目录 | 功能 |
|----------|------|
| `daohang_dianti/` | 语音→导航到电梯 |
| `daohang_weishengjian/` | 语音→导航到卫生间 |
| `handshake/`, `yongbao/`, `qin/` | 手臂动作（握手/拥抱/亲吻） |
| `nihao/` | 打招呼（手臂+人脸识别） |
| `identify/`, `camera/` | 人脸识别/拍照 |
| `music/` | 播放音乐 |
| `timer/`, `calendar/` | 定时器/日历查询 |
| `amap/`, `search/`, `recipe/` | 高德/搜索/菜谱查询 |

## 部署拓扑
```
宿主机 (ROS2 Foxy)                          容器 g1slam (ROS1 Noetic)
/home/unitree/HongTu  --映射--  /root/HongTu
                                               
    ros1_bridge                          catkin workspace
         
外部 PC (rviz2 可视化)
```
**网络**：G1 开发板 `192.168.123.164`；Livox 雷达 `192.168.123.120`

## 修改偏好
1. **ROS 侧**：优先改 launch/YAML，避免 C++ 硬编码
2. **livox_ros_driver2**：视为 vendored 上游，只改 `config/*.json` 与 launch
3. **MID360 倒装**：修改 `MID360_config.json` 的 `extrinsic_parameter.roll: 180.0`
4. **外参调整**：修改 `mapping.yaml`/`localize.yaml` 的 `lio_builder/imu_ext_rot`, `imu_ext_pos`

## 排障速查
| 现象 | 检查点 |
|------|--------|
| 点云倒置 | `rostopic echo /livox/imu` 检查 z 轴加速度方向 |
| TF 断裂 | RViz 查看 `maplocalbodybase_link` 链路 |
| /scan 无数据 | 检查 `point_to_scan.launch` 的 `min_height/max_height` |
| 重定位无响应 | `rosservice call /slam_reloc_check` 验证 |

**详细排障**：见 `docs/FAST-LIO2排障清单.md`
