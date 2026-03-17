# G1 导航“有 cmd_vel 但机器人不动”排障与参数整理

> 适用场景：ROS1 move_base/TEB 导航链路存在 `/cmd_vel` 输出，但 Unitree G1 在运动模式下不响应或仅在速度较大时才迈步。

## 1. 现象与结论

### 1.1 典型现象
- RViz 可规划出路径，`/cmd_vel` 持续有数据，但机器人不动。
- 运行官方示例 `g1_loco_client_example.py`，输入 `3/5` 等选项机器人能走/能转。
- 实测：只有当
  - `|vx| >= 0.2`（线速度）或
  - `|wz| >= 0.3`（角速度）
  机器人才会明显运动。
- 当速度超过阈值后，即便另一个分量较小，也可能出现“边走边转/边转边走”的耦合现象。

### 1.2 核心结论
- **SDK/机器人侧存在“最小可运动速度阈值（deadband）”**：规划器输出的小速度（例如 0.05~0.15）对 G1 来说等价于 0。
- 仅调整规划器的 `min_vel_x` 不一定能直接在机器人上体现为“更小步距/更慢速度”，因为机器人本体无法执行过小速度。
- 若中间链路存在平滑/平均（例如 EMA smoother、窗口平均），会把规划器的最小速度进一步“抹小”，导致再也迈不开步。

---

## 2. 诊断过程要点（可复用）

### 2.1 先证明“SDK链路正常”
- 使用官方示例验证接口/网络没问题：
  - `unitree_sdk2_python/example/g1/high_level/g1_loco_client_example.py`
  - `python3 g1_loco_client_example.py eth0` → 按提示 `Enter` → 输入 `3`（前进）

### 2.2 再做最小闭环测试（不依赖导航）
- 新增测试脚本：
  - `unitree_sdk2_python/example/g1/high_level/g1_control_test.py`
- 用 `rostopic pub` 手动发 cmd_vel，验证“不同速度阈值下是否能动”。

常用命令（注意 YAML 逗号问题）：
- 前进：
  - `rostopic pub -r 2 /cmd_vel geometry_msgs/Twist "linear: {x: 0.3}"`
- 原地转：
  - `rostopic pub -r 2 /cmd_vel geometry_msgs/Twist "angular: {z: 0.3}"`
- 同时前进+转向（两种写法均可）：
  - `rostopic pub -r 2 /cmd_vel geometry_msgs/Twist "linear: {x: 0.2} angular: {z: 0.3}"`
  - `rostopic pub -r 2 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"`
- 停止：
  - `rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "{}"`

### 2.3 再接入导航链路定位“速度被抹小”的环节
关键做法：同时观察
- 规划器输出（如 `raw_cmd_vel`）
- smoother 后输出（如 `/cmd_vel`）
- 下发给 SDK 的“映射后速度”（g1_control 打印）

---

## 3. 关键问题与修复点

### 3.1 `Move()` 返回值为何是 `None`
- `LocoClient.Move()` 内部调用 `SetVelocity(...)`，但 `Move()` 本身不返回 code；所以日志里看到 `Move result: None` 是正常的。
- 为了可观测性，控制侧改为直接调用 `SetVelocity(...)` 并打印返回码（`code=0` 表示成功）。

### 3.2 “规划器速度小导致不动”的处理策略
有两条路：

**A. 规划器层面（让输出不要落入机器人 deadband）**
- 在 TEB 参数里设置最小速度（示例）：
  - `min_vel_x: 0.2~0.3`
  - `min_vel_theta: 0.3`
  - `max_vel_x_backwards` 需足够大，否则“走过头”无法倒退脱困。

**B. 控制器层面（把小速度映射到最小可运动速度）**
- 在 `g1_control.py` 中做：
  - 死区过滤（lin/ang deadband）
  - 最小速度映射（保持方向）
  - 倒退专用阈值与限幅（避免“小负速度”被扩大成固定大倒退，且允许必要倒退脱困）
  - 角速度符号保持（减少左右摇摆）

实践经验：**A+B 一起用最稳**。

---

## 4. 最终落地修改（文件级）

### 4.1 控制器：`g1_control.py`
路径：
- `unitree_sdk2_python/example/g1/high_level/g1_control.py`

关键能力：
- 接收 `/cmd_vel` 并缓冲平均
- 定时器周期性调用 SDK（避免高频压垮）
- 对速度做 deadband + 最小可运动速度映射 + 倒退策略
- 用 `SetVelocity(vx, vy, wz, duration)` 下发并打印返回码
- 日志打印 `raw_avg -> mapped` 便于定位“速度被抹小”的环节

建议运行参数（根据现场调）：
- 下发频率与步长（推荐组合）：
  - `~sdk_call_interval: 0.2`（5Hz）
  - `~command_duration: 0.2`（每次只持续 0.2s，减少走过头）
- 最小速度阈值（按实测）：
  - `~min_lin_speed: 0.2~0.35`
  - `~min_ang_speed: 0.3`
- 倒退（脱困关键）：
  - `~allow_backwards: true`
  - `~min_backwards_speed: 0.2~0.3`
  - `~max_backwards_speed: 0.3`
- 抖动抑制：
  - `~ang_sign_hold_sec: 0.8~1.2`

### 4.2 测试脚本：`g1_control_test.py`
路径：
- `unitree_sdk2_python/example/g1/high_level/g1_control_test.py`

用途：
- 仅订阅 `/cmd_vel`，不依赖导航话题，快速验证阈值和网络链路。

### 4.3 规划器：TEB 参数
路径：
- `G1Nav2D/src/movebase/param/teb_local_planner_params.yaml`

已做过的典型调整方向：
- 最小速度：`min_vel_x/min_vel_theta`（例如 0.3）
- 倒退上限：`max_vel_x_backwards`（例如 0.3，避免走过头无法倒退）
- 提速：提高 `max_vel_theta`、`acc_lim_x`、`acc_lim_theta`、`weight_optimaltime`
- 降低到点精度：增大 `xy_goal_tolerance`、`yaw_goal_tolerance`

验证参数是否生效（在机器人/容器内）：
- `rosparam get /move_base/TebLocalPlannerROS/min_vel_x`
- `rosparam get /move_base/TebLocalPlannerROS/min_vel_theta`
- `rosparam get /move_base/TebLocalPlannerROS/max_vel_x_backwards`

---

## 5. 常见坑总结

1) **`rostopic list` 看不到 `/cmd_vel`**：只有当发布者开始发布或订阅者存在时才会出现。
2) **`rostopic pub` YAML 语法错误**：不要在同一层字段之间写逗号（`,`）。
3) **速度“看起来没变”**：你看到的可能是控制器映射后的速度，不是 planner 原始输出。
4) **中间平滑把最小速度抹小**：EMA/平均会导致 `0.3` 变成 `0.18`，从而迈不开步。
5) **走过头抖动**：需要倒退能力 + 更短 duration + 更松 goal tolerance，而不是“更小速度”。

---

## 6. 推荐的最终目标配置（经验配方）

如果目标是“走快一点、到点不那么精确”：
- TEB：
  - `xy_goal_tolerance: 0.5`
  - `yaw_goal_tolerance: 0.35`
  - `weight_optimaltime: 5`（可 5~10）
  - `acc_lim_x/acc_lim_theta: 0.8`
  - `max_vel_theta: 0.7`
  - `min_vel_x/min_vel_theta: 0.3`
- 控制器：
  - `sdk_call_interval: 0.2`
  - `command_duration: 0.2`
  - `min_lin_speed/min_backwards_speed: 0.3`

---

## 7. 关联入口
- 导航入口（用于确认参数加载顺序）：
  - `G1Nav2D/src/fastlio2/launch/navigation.launch`

（完）
