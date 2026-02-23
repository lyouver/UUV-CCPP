# UAV Intent-MPC 适配到 UUV（RexROV）详细改造说明

本文档是这次迁移的“落地版”说明，目标是把原来偏无人机接口的 Intent-MPC 思路，接入你当前的 UUV 仿真与控制链，且可 A/B 并行验证。

## 1. 迁移目标与边界

1. 目标：复用 Intent-MPC 的局部规划能力（`trajectory_planner/mpcPlanner`），替换或并行验证你现有的局部避障策略。
2. 不改：保留 UUV 官方控制链（`rov_pid_controller`、推进器管理、UUV 控制消息体系）。
3. 核心做法：新增“UUV 适配层”节点，把 UAV 风格输入输出转换为 UUV 可消费的输入输出。

## 2. 无人机代码与 UUV 代码的关键差异

1. 动力学与控制接口不同：UAV 常见 `Target`/自定义消息，UUV 侧是 `uuv_control_msgs/Trajectory` 或 `InitWaypointSet` 服务。
2. 坐标系约束更严格：UUV 控制链要求 `world` 或 `world_ned`，空 frame 或非法 frame 会直接出错。
3. 传感输入不同：UUV 工程里动态障碍主要来自 `onboard_detector` 的 `MarkerArray`，不是 UAV 常见点云/目标列表结构。
4. 控制器运行逻辑不同：UUV 的 `DPControllerLocalPlanner` 有 `station_keeping`、`trajectory_running`、`smooth_approach` 状态机。

## 3. 迁移后的整体架构

1. 输入：
   - `/rexrov/pose_gt`（当前状态）
   - `/onboard_detector/velocity_visualizaton`（动态障碍）
   - 全局航点 YAML（覆盖路径）
2. 中间：
   - `map_manager::occMap` 做占据/碰撞查询
   - `trajectory_planner::mpcPlanner` 做局部轨迹优化
3. 输出（两种）：
   - 正常：调用 `/rexrov/start_waypoint_list`（waypoint 模式）
   - 兜底：发布 `/rexrov/dp_controller/input_trajectory`（trajectory 模式）

## 4. 第一步：把 MPC 相关包放进 catkin 工作区

你当前工作区使用的是 `/home/tb/dave_ws/src`。迁移时保证以下包在 `src/` 根目录可编译：

1. `trajectory_planner`
2. `global_planner`
3. `dynamic_predictor`
4. `map_manager`
5. `onboard_detector`

说明：

1. `uuv_mpc_adapter` 在 `src/EROAS-main/uuv_mpc_adapter`。
2. 适配节点通过 `catkin` 依赖链接上面几个包，不需要单独再搞一套 UAV 节点链。

## 5. 第二步：处理 OctoMap 相关依赖/编译开关

### 5.1 包内开关位置

1. `src/trajectory_planner/CMakeLists.txt`：
   - `TRAJECTORY_PLANNER_BUILD_OCTOMAP_TOOLS`
   - 影响演示节点：`poly_RRT_node`、`poly_RRTStar_node` 等
2. `src/global_planner/CMakeLists.txt`：
   - `GLOBAL_PLANNER_BUILD_OCTOMAP_TOOLS`
   - 影响演示节点：`rrt_interactive_node`、`rrt_star_interactive_node`

### 5.2 是否必须安装 OctoMap 运行时

1. 核心 MPC 链路不依赖这些“交互演示节点”。
2. 若你要把这些节点也编过并跑起来，安装：
   - `ros-noetic-octomap-ros`
   - `ros-noetic-octomap-msgs`
   - `ros-noetic-octomap`

## 6. 第三步：新增 UUV 适配节点 `uuv_mpc_adapter`

### 6.1 包与依赖

1. `src/EROAS-main/uuv_mpc_adapter/CMakeLists.txt`
2. `src/EROAS-main/uuv_mpc_adapter/package.xml`

依赖包含：

1. `trajectory_planner`
2. `global_planner`
3. `dynamic_predictor`
4. `map_manager`
5. `uuv_control_msgs`

### 6.2 节点职责

文件：`src/EROAS-main/uuv_mpc_adapter/src/mpc_uuv_adapter_node.cpp`

1. 读取全局航点 YAML，稠密化路径。
2. 构建 `occMap` + `mpcPlanner`（使用节点私有参数空间）。
3. 订阅 odom、障碍信息，更新 MPC 状态。
4. 输出到 UUV 控制入口（waypoint 服务或 trajectory 话题）。

## 7. 第四步：输入侧适配细节

### 7.1 Odom 适配

1. 输入：`/rexrov/pose_gt`
2. 映射：
   - 位置 -> `curr_pos`
   - 线速度 -> `curr_vel`

### 7.2 动态障碍适配

1. 输入：`/onboard_detector/velocity_visualizaton`（`visualization_msgs/MarkerArray`）
2. 解析策略：
   - 位置：`marker.pose.position`
   - 速度：从 `marker.text` 提取 `Vx/Vy`（正则）
   - 尺寸：`marker.scale.x` 或默认半径
3. 超时剔除：超过 `obstacle_timeout` 的障碍不参与 MPC。

### 7.3 全局路径适配

1. 输入：覆盖任务航点 YAML。
2. 处理：
   - 读取 `point: [x,y,z]`
   - 稠密化到 `waypoint_resolution`
   - 转换为 `nav_msgs/Path` 供 MPC 初始化

## 8. 第五步：MPC 规划链路适配

关键调用顺序（在 `planCB`）：

1. 首次调用 `mpc_->updatePath(path_msg_, mpc_dt_)`
2. 每周期调用 `mpc_->updateCurrStates(curr_pos, curr_vel)`
3. 每周期调用 `mpc_->updateDynamicObstacles(ob_pos, ob_vel, ob_size)`
4. `mpc_->makePlan()` 求解
5. 成功则 `getTrajectory(traj)`；失败则回退到“沿全局路径前推的 fallback 轨迹”

说明：

1. 这样做保证“即使 MPC 瞬时失败，机器人也不至于停死”。

## 9. 第六步：输出侧适配（最关键）

### 9.1 Waypoint 模式（优先）

1. 通过服务 `/rexrov/start_waypoint_list` 下发 `InitWaypointSet`。
2. 重点参数：
   - `start_now = true`
   - `interpolator`（当前配置 `lipb`）
   - `max_forward_speed`
   - `radius_of_acceptance`
3. 发送频率由 `waypoint_update_period` 限制，避免过高频重置。

### 9.2 Trajectory 话题模式（兜底）

1. 当 waypoint 调用失败时，自动回退发布 `uuv_control_msgs/Trajectory`。
2. 发布到 `output_trajectory_topic`，默认 `/rexrov/dp_controller/input_trajectory`。

### 9.3 服务调用稳定性改造

1. 服务客户端使用非持久连接（`persistent=false`）。
2. 失败后重建 service client，避免连接陈旧导致连续失败。

## 10. 第七步：控制器侧兼容修复

文件：`src/EROAS-main/uuv_simulator/uuv_control/uuv_trajectory_control/src/uuv_control_interfaces/dp_controller_local_planner.py`

做了与适配相关的兼容处理：

1. 空 `WaypointSet` 也写入合法 `header.frame_id`（防止 marker 断言）。
2. `start_waypoint_list` 回调增强异常处理和时间字段处理，避免服务端回调异常导致客户端 `failed calling`。
3. 高频 waypoint 更新时，`smooth_approach` 不重复触发（仅 `start_now=False` 才开），减少“原地打转/追航向”。

## 11. 第八步：Launch 级集成与 A/B 并行验证

文件：`src/EROAS-main/example/src/launch/compact_rexrov_with_trajectory.launch`

新增 `planner_backend` 三种模式：

1. `rule`：仅原规则规划器
2. `mpc`：MPC 接管
3. `dual`：并行 A/B（规则控制真实链路，MPC 发调试轨迹）

MPC 参数从 launch 直接注入：

1. `mpc_max_vel`
2. `mpc_max_acc`
3. `mpc_plan_period`

同时显式锁定 PID 为官方默认参数，避免迁移时控制器参数漂移。

## 12. 第九步：MPC 参数文件落地

文件：`src/EROAS-main/example/src/config/local_mpc_adapter.yaml`

包含四类参数：

1. I/O 话题与路径参数
2. 轨迹/速度/加速度/周期参数
3. `occupancy_map/*` 参数（给 `occMap`）
4. `mpc_planner/*` 参数（给 `mpcPlanner`）

补充：

1. `waypoint_interpolator` 当前为 `lipb`（用于抑制急剧航向抖动）。

## 13. 第十步：map_manager 兼容修复

文件：`src/map_manager/include/map_manager/occupancyMap.cpp`

1. 当 `prebuilt_map_directory` 为空字符串、`No`、`None` 时直接跳过 PCD 加载。
2. 避免日志反复出现 `[pcl::PCDReader::read] Could not find file ''`。

## 14. 第十一步：编译与启动标准流程

### 14.1 编译

```bash
cd /home/tb/dave_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg map_manager uuv_trajectory_control uuv_mpc_adapter
source devel/setup.bash
```

### 14.2 启动主链路（MPC）

```bash
roslaunch eroas_example compact_rexrov_with_trajectory.launch planner_backend:=mpc --screen
```

### 14.3 启动动态障碍检测

```bash
roslaunch onboard_detector run_detector.launch show_rviz:=false publish_map_frame_tf:=false
```

## 15. 第十二步：验证清单（上线前必须过）

```bash
rosnode list | grep -E "mpc_uuv_adapter|rov_pid_controller|detector_node|local_avoidance_planner"
rosservice list | grep /rexrov/start_waypoint_list
rostopic hz /onboard_detector/velocity_visualizaton
rostopic hz /rexrov/dp_controller/waypoints
rostopic hz /rexrov/dp_controller/input_trajectory
```

预期：

1. `mpc` 模式下，`mpc_uuv_adapter` 与 `rov_pid_controller` 在。
2. `start_waypoint_list` 服务存在。
3. 检测器话题有频率。
4. 正常优先看到 waypoints 更新；服务失败时能看到 trajectory 兜底更新。

## 16. 常见问题与定位思路

### 16.1 `failed calling /rexrov/start_waypoint_list`

1. 先看服务是否存在。
2. 再看 `rov_pid_controller` 日志是否有异常堆栈。
3. 如果服务偶发失败，确认是否触发了 trajectory 兜底发布。

### 16.2 机器人原地旋转

1. 先检查 `waypoint_interpolator`  `cubic` 改为 `lipb`。
2. 检查是否在高频重置 waypoint 且 `smooth_approach` 被重复开启。
3. 检查速度上限与加速度上限是否过高导致追踪振荡。

### 16.3 机器人反应慢

1. 先确认不是 PID 参数偏离官方值。
2. 再调 `mpc_plan_period`、`max_vel`、`max_acc`。
3. 最后再调 `waypoint_update_period`。

## 17. 本次迁移改动文件总览

1. `src/EROAS-main/uuv_mpc_adapter/CMakeLists.txt`
2. `src/EROAS-main/uuv_mpc_adapter/package.xml`
3. `src/EROAS-main/uuv_mpc_adapter/src/mpc_uuv_adapter_node.cpp`
4. `src/EROAS-main/example/src/config/local_mpc_adapter.yaml`
5. `src/EROAS-main/example/src/launch/compact_rexrov_with_trajectory.launch`
6. `src/EROAS-main/uuv_simulator/uuv_control/uuv_trajectory_control/src/uuv_control_interfaces/dp_controller_local_planner.py`
7. `src/map_manager/include/map_manager/occupancyMap.cpp`

## 18. 建议的迭代方式

1. 永远先 `dual` 并行，再 `mpc` 接管。
2. 每次只改一类参数（速度上限或周期或插值器），避免多因素叠加。
3. 用固定场景复现实验，记录同一批话题频率与误差，再比较。

