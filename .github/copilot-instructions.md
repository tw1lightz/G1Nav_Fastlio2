# HongTu：AI coding agent 指南

## 仓库结构（先看这里）
- ROS1 Noetic 2D 导航/SLAM：G1Nav2D/（catkin 工作区）
- 语音客户端 + MCP 工具（会触发导航脚本）：PythonProject/py-xiaozhi-main/
- 宇树运动控制 SDK：unitree_sdk2_python/

## ROS 关键链路（G1Nav2D）
- 主入口：G1Nav2D/src/fastlio2/launch/navigation.launch（定位+2D地图+move_base+scan 管线）
- 话题管线（不要只改一处）：
  - tool/body2any_pointcloud.launch：订阅 velodyne_points；发布 /base_cloud（在 launch 内 remap 到 base_link_cloud）
  - pointcloud_to_laserscan/launch/point_to_scan.launch：cloud_in=/base_link_cloud → /scan（高度阈值 min/max_height 在此）
  - fastlio2/launch/gridmap_load.launch：map_server 发布 /map_2d（注意 topic 名不能数字开头）
- 参数入口：fastlio2/config/mapping.yaml 与 fastlio2/config/localize.yaml；点云半径裁剪参数在 fastlio2/README.md 里（lio_builder/*_point_range）。
- 建图/导航（Ubuntu/容器内）：
  - 依赖：Livox-SDK2 + livox_ros_driver2-master（MID360_config.json 配 IP）
  - 编译：cd G1Nav2D/src/livox_ros_driver2-master && ./build.sh ROS1；cd G1Nav2D && catkin_make
  - 运行：source G1Nav2D/devel/setup.bash；roslaunch fastlio mapping.launch / navigation.launch

## 地图与重定位（fastlio）
- 保存 3D 点云：rosservice call /save_map "{save_path: '/abs/path/map.pcd', resolution: 0.0}"（resolution 当前未使用）
- navigation.launch 默认读取：pcd_map=/root/HongTu/test2.pcd，map2d_yaml=/root/HongTu/map/test2.yaml（可用 launch 参数覆盖）
- 外部 PC 跑 RViz：按 fastlio2/README.md 设置 ROS_MASTER_URI/ROS_IP，优先用 TF/Map(/map_2d)/LaserScan(/scan)/Odometry(/slam_odom) 排查。

## 真实部署拓扑（容器优先）
- 宿主机：ROS2 Foxy（无可视化界面）；容器 ros1_noetic_hongtu：ROS1 Noetic（本项目当前以 Noetic 版本为主）。
- 目录映射：宿主机 /home/unitree/HongTu ↔ 容器 /root/HongTu（写路径时优先按容器路径描述）。
- 可视化与跨域通信：通常通过 ros bridge 把容器内 ROS1 话题桥到 ROS2 Foxy；外部 PC 通过 ROS2 DDS 订阅/发布这些 ROS2 话题做可视化/调试。
- 外部 PC 可视化建议：优先用 ROS2 的 rviz2 订阅桥接后的 ROS2 话题（而不是 ROS1 rviz 直连容器内 ROS Master）。
- 常用网络信息（排障用）：G1 开发板 IP=192.168.123.164；Livox IP=192.168.123.120。

## 语音 → MCP → 导航脚本（PythonProject）
- 关键词路由：PythonProject/py-xiaozhi-main/src/application.py（例如“电梯/楼梯”→ call_tool("self.elevator_stairs.trigger")）
- 工具注册：PythonProject/py-xiaozhi-main/src/mcp/mcp_server.py（add_common_tools 里 get_*_manager().init_tools）
- 导航脚本链路（大量硬编码路径，优先改这些而不是动音频主流程）：
  - MCP 工具实现：py-xiaozhi-main/src/mcp/tools/daohang_dianti/tools.py（python_path/script_path）
  - 中转脚本：PythonProject/daohang/daohang-dianti.py（Popen 拉起 point_nav/*.py）
  - 目标点脚本：PythonProject/point_nav/point*.py（目标点/音频文件路径等）
  - 注意：仓库里常见硬编码是 /home/zhuo/...；在容器化部署中建议统一改为 /root/HongTu/...（或通过环境变量/配置集中管理）。

## 运控（Unitree）
- 示例入口：unitree_sdk2_python/example/g1/high_level/（例如 python3 g1_control.py <网口名>；网口用 ifconfig 查询）

## 修改偏好（本仓库约定）
- ROS 侧优先改 launch/YAML（topic/frame/阈值/路径），尽量避免在 C++ 里硬编码新常量。
- livox_ros_driver2-master 视为上游 vendored：优先改其 config/*.json 与 launch，而不是重写驱动代码。
