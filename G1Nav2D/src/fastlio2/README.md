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