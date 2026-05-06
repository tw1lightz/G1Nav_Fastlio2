## 部署

### 克隆仓库
  ``` bash
  git clone https://github.com/yuanqizhiti/HongTu.git
  ```
### 复用旧容器（非首次部署）
```bash
映射路径：/home/unitree/HongTu→/root/HongTu

# 启动已停止的容器
docker start ros1_noetic_hongtu

# 进入容器交互终端
docker exec -it ros1_noetic_hongtu /bin/bash
```
### 2D导航
- 安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
    ```bash
    sudo apt install cmake
    ```

    ```bash
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```

- 更改雷达ip及地图保存路径
  ``` bash
  # 修改本机与雷达ip
  cd HongTu/G1Nav2D/src/livox_ros_driver2-master/config/
  gedit MID360_config.json
  
  # 修改地图保存路径，将该文件下最底部路径改为自己的电脑
  cd HongTu/G1Nav2D/src/fastlio2/src/
  gedit map_builder_node.cpp
  ```

- 编译程序
  ``` bash
  cd /root/HongTu/G1Nav2D/src/livox_ros_driver2-master/

  # 清理旧编译文件
  rm -rf ../../build/ ../../devel/ ../../install/

  # 执行编译
  ./build.sh ROS1
  catkin_make -DROS_EDITION=ROS1 --pkg livox_ros_driver2
  catkin_make -DROS_EDITION=ROS1 --pkg fastlio
  catkin_make -DROS_EDITION=ROS1
  ```

- 安装依赖包
  ``` bash
  sudo apt install ros-noetic-teb-local-planner ros-noetic-global-planner ros-noetic-costmap-server
  ```

- 建图及保存
  ``` bash
  # 建图
  cd HongTu/G1Nav2D/
  source devel/setup.bash
  roslaunch fastlio mapping.launch
  
  # 打开新终端
  cd HongTu/G1Nav2D/
  source devel/setup.bash
  # 保存地图，自定义路径及地图名称
  rosrun map_server map_saver map:=/projected_map -f /home/nvidia/mymap
  ```

- 编辑地图
  ``` bash
  # 打开地图，利用Map Eraser Tool修改地图，ctrl+加号或减号可修改画笔大小，保存地图
  source devel/setup.bash
  roslaunch ros_map_edit map_edit.launch
  ```

- 开启导航
  ``` bash
  #修改地图路径
  cd HongTu/G1Nav2D/src/fastlio2/config/
  gedit gridmap_load.launch
  
  # 启动导航，启动导航后需自行按照雷达位置重定位
  cd HongTu/G1Nav2D/
  source devel/setup.bash
  roslaunch fastlio navigation.launch
  ```

- 启动运控  
安装unitree_sdk2_python参考[宇树官方文档](https://github.com/unitreerobotics/unitree_sdk2_python.git)
  ``` bash
  # 打开新终端，网口可通过ifconfig命令查询自行更改
  cd HongTu/unitree_sdk2_python/example/g1/high_level/
  python3 g1_control.py 网口
  ```
在rviz中发布目标点即可自主导航

### 语音交互
基于[pyxiaozhi](https://github.com/huangjunsen0406/py-xiaozhi)，ubuntu20.04默认python版本不符合，安装小智需要配置虚拟环境。
- 基础要求
    Python版本：3.9 - 3.12
    操作系统：Windows 10+、macOS 10.15+、Linux
    音频设备：麦克风和扬声器设备
    网络连接：稳定的互联网连接（用于AI服务和在线功能）

- 安装依赖
  ``` bash
  cd PythonProject/py-xiaozhi-main/
  pip install -r requirements.txt
  ```
  
- 语音导航至目标点简易版
  1. 全局搜索关键词“电梯”，将所有“电梯”替换成你需要的关键词，例如“卧室”、“卫生间”等。
  2. 在Pythonproject/point_nav/point2.py修改改点坐标，修改位置在该程序最底部。（坐标可以通过导航发布目标点时，监听/move_base/goal话题获取，手动输入，当前为测试版本，每个目标点为不同的启动程序）
 
- 导航到目标点MCP服务
  ``` bash
  #以导航至电梯目标点为例
  # 在该文件的第1232行，修改或添加导航至目标点的关键词
  PythonProject/py-xiaozhi-main/src/application.py
  
  # 在该文件的第334行，修改或添加mcp服务的注册信息
  PythonProject/py-xiaozhi-main/src/mcp/mcp_server.py
  
  #在该文件的第15至第17行，选择该mcp服务拉起的python程序，以及启动该程序的编译器路径
  PythonProject/py-xiaozhi-main/src/mcp/tools/daohang_dianti/tools.py
  
  #在该文件的第10至第13行，选择拉起的导航点程序，以及启动该程序的编译器路径
  PythonProject/daohang/daohang-dianti.py
  
  #在该文件修改目标点的坐标
  PythonProject/point_nav/point1.py
  ```

- 启动语音程序
  ``` bash
  cd PythonProject/py-xiaozhi-main/
  python3 main.py
  ```
实现语音交互导航需要同时开启语音、运控、导航。

