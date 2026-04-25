# 毕业论文主路线说明：任务级全局覆盖路径 + 在线局部MPC避障

本文档面向毕业论文写作，给出当前工程主路线的完整技术链路。结构采用“双层全局路径主线”：

- 上层（任务级、离线/半离线）：`underwater_coverage_planning` 生成覆盖任务的全局路径（waypoints）。
- 下层（在线、实时）：`uuv_mpc_adapter + trajectory_planner + map_manager + onboard_detector` 在仿真运行中跟踪全局路径，并在局部层执行三维避障。

文档强调算法与原理，代码锚点仅保留核心入口，避免碎片化堆砌。

---

## 1. 系统总览

### 1.1 主链路（离线 + 在线）

```mermaid
flowchart LR
  A[地形DAE/配置] --> B[任务级全局覆盖规划<br/>underwater_coverage_planning]
  B --> C[*_ros_waypoints.yaml]
  C --> D[global_waypoint_file]
  D --> E[uuv_mpc_adapter<br/>全局路径稠密化+平滑]
  E --> F[模式机<br/>GLOBAL_PASS / LOCAL_MPC / REJOIN_HOLD]
  F --> G[trajectory_planner(MPC)]
  H[onboard_detector] --> G
  I[occupancy_map(map_manager)] --> G
  G --> J[/rexrov/dp_controller/input_trajectory]
  J --> K[UUV控制器执行]
```

### 1.2 两段流程的工程关系

- 第一段是“生成任务路径”：覆盖规划器输出 `*_ros_waypoints.yaml`。
- 第二段是“在线执行和避障”：适配器读取 `global_waypoint_file`，每个周期结合动态检测与占据地图进行局部MPC重规划。
- 当前主仿真启动链路中，覆盖规划器通常作为上游单独运行；在线阶段默认直接读 waypoint 文件。

---

## 2. 任务级全局路径生成（`underwater_coverage_planning`）

本章对应论文中的“全局任务规划/覆盖路径规划”部分。核心是 SIP 风格迭代覆盖优化，不是简单 waypoint 读取。

### 2.1 地形建模与可查询高度场

输入是地形网格（DAE）与规划配置（区域边界、尺度、分辨率等）。

流程：

1. 加载三角网格并做尺度变换与高度偏移（`mesh_scale_factor`, `mesh_z_offset`）。
2. 按配置裁剪关注区域（`crop_x/y_*`）。
3. 将三角面栅格化到规则 XY 网格，得到离散地形高度图 `h(i,j)`。
4. 查询任意点高度时采用双线性插值（必要时用邻域保守上界回退）。

双线性插值形式：

\[
z_{\text{terrain}}(x,y)=\text{Bilinear}(h_{00},h_{10},h_{01},h_{11};w_x,w_y)
\]

边段离地间隙定义：

\[
c(\mathbf{p})=p_z-z_{\text{terrain}}(p_x,p_y),\quad
c_{\min}(e_{ij})=\min_{\mathbf{p}\in e_{ij}} c(\mathbf{p})
\]

这一步为后续“可行边判定”提供几何可达性基础。

### 2.2 视点采样（Coverage Viewpoints）

对每个候选地形面中心 \(\mathbf{c}\) 和法向 \(\mathbf{n}\)，视点构造为：

\[
\mathbf{p}=\mathbf{c}\pm \mathbf{n}\cdot s+\boldsymbol{\delta}
\]

其中：

- \(s\)：standoff 距离（由 `base_standoff_distance` 与迭代扰动决定）
- \(\boldsymbol{\delta}\)：切平面内侧向扰动

候选视点必须满足：

1. XY 在地形网格有效范围内（避免越界孤点）。
2. Z 满足深度边界 \(z_{\min}\le z \le z_{\max}\)。
3. 视点离地间隙满足：
\[
c(\mathbf{p}) \ge c_{\text{pref}}
\]
4. 与已保留视点的最小间距约束（`min_viewpoint_spacing`）。

### 2.3 有向边可行性与代价矩阵

对任意有向边 \(i\to j\)：

1. 沿线段按步长采样 `edge_sample_step`。
2. 若任一点越界或 \(c_{\min}(e_{ij}) < c_{\text{terrain}}\)（`terrain_clearance`），则不可行。

代价矩阵定义：

\[
C_{ij}=
\begin{cases}
\|\mathbf{p}_j-\mathbf{p}_i\|_2, & \text{edge feasible}\\
M, & \text{edge infeasible}
\end{cases}
\]

其中 \(M=\texttt{fallback\_large\_cost}\)。

### 2.4 巡回顺序优化（ATSP）

有向代价矩阵构造后，求解访问顺序：

- 优先使用 LKH（ATSP）。
- 不可用时退化为 nearest-neighbor + 2-opt。
- `open_tour` 场景下会将闭环中最大代价边作为“隐式断边”，转为开链输出。

论文中可表述为：

\[
\min_{\pi}\sum_{k} C_{\pi_k,\pi_{k+1}}
\]

其中 \(\pi\) 为视点访问序列。

### 2.5 SIP 迭代重采样（核心）

初始顺序求得后，进行多轮局部重采样。对路径中第 \(i\) 个视点，构造候选集 \(\mathcal{V}_i\)（standoff 增减 + 侧向抖动），局部优化：

\[
\mathbf{v}_i^*=
\arg\min_{\mathbf{v}\in\mathcal{V}_i}
\Big( C_{i-1,\mathbf{v}} + C_{\mathbf{v},i+1} \Big)
\]

仅接受两段边都可行的候选。全体节点循环后进入下一轮，典型迭代参数是 `resample_iterations`。

### 2.6 轨迹导出到 ROS waypoints

按优化顺序导出 `*_ros_waypoints.yaml`。中间计算采用：

\[
dt = \max(t_{xy}, t_z) + t_{\text{turn}}
\]
\[
t_{xy}=\frac{d_{xy}}{v_{xy,\max}},\quad
t_z=\frac{|dz|}{v_{z,\max}},\quad
t_{\text{turn}}=\frac{|\Delta\psi|}{\dot\psi_{\max}}
\]

最终写入 ROS waypoint 结构，供在线执行器读取。

---

## 3. 全局路径到在线执行轨迹

### 3.1 数据衔接

在线适配器读取 `global_waypoint_file`，解析 `point: [x,y,z]` 序列，并构造全局参考路径。

### 3.2 全局路径几何处理

在线阶段对全局路径执行两步：

1. 线性稠密化（按 `waypoint_resolution`）
2. LIPB平滑（线段 + 二次Bezier转角过渡）

转角段可写为二次Bezier：

\[
\mathbf{b}(t)=(1-t)^2\mathbf{p}_0+2(1-t)t\mathbf{p}_1+t^2\mathbf{p}_2,\quad t\in[0,1]
\]

这样得到可跟踪、曲率更平滑的在线全局参考。

### 3.3 在线参考到控制轨迹

- 适配器周期性输出 `uuv_control_msgs/Trajectory`。
- 速度由相邻离散点差分得到，航向由路径切向估计（`yaw_from_path`）。
- 若局部MPC暂时不可用，退化为沿全局参考的 fallback 轨迹，保证系统持续输出可执行命令。

---

## 4. 模式机与触发逻辑（GLOBAL_PASS / LOCAL_MPC / REJOIN_HOLD）

### 4.1 三状态定义

- `GLOBAL_PASS`：沿全局轨迹主通行。
- `LOCAL_MPC`：进入局部避障MPC。
- `REJOIN_HOLD`：局部结束后短暂保持并准备回归全局。

### 4.2 障碍相关判据

对每个障碍，先做时效过滤：

\[
t_{\text{now}}-t_{\text{obs}} \le t_{\text{timeout}}
\]

再计算二维净空：

\[
d_{\text{clear},xy}=\|\mathbf{p}_{obs,xy}-\mathbf{p}_{robot,xy}\|-(r_{obs}+r_{robot})
\]

仅保留前向窗口内、非后向忽略区内的障碍（由全局路径切向投影判定）。

### 4.3 状态切换条件（可写成论文逻辑）

- `GLOBAL_PASS -> LOCAL_MPC`：
\[
d_{\min,\text{clear},xy}\le d_{\text{trigger}}
\]

- `LOCAL_MPC -> REJOIN_HOLD`：
障碍已持续清空（或局部超时）且机器人已靠近全局路径：
\[
d_{\text{to-global}}\le d_{\text{rejoin-path}}
\]

- `REJOIN_HOLD -> GLOBAL_PASS`：
保持时间到且靠近全局路径。

- `REJOIN_HOLD -> LOCAL_MPC`（紧急重入）：
\[
d_{\min,\text{clear},xy}\le d_{\text{emergency}}
\]

---

## 5. 局部MPC建模与求解

### 5.1 离散状态与控制

当前实现是离散线性模型，优化变量包含位置、速度与松弛相关状态。可在论文中写为：

\[
\mathbf{x}_{k+1}=A\mathbf{x}_k+B\mathbf{u}_k
\]

其中平动主状态是 \((x,y,z,v_x,v_y,v_z)\)，控制主量是 \((a_x,a_y,a_z)\)，并含动态/静态障碍松弛变量通道。

### 5.2 目标函数

标准二次型：

\[
J=\sum_{k=0}^{N}\|\mathbf{x}_k-\mathbf{x}_k^{ref}\|_Q^2
+\sum_{k=0}^{N-1}\|\mathbf{u}_k\|_R^2
\]

其中 \(Q,R\) 为对角权重矩阵。

### 5.3 基础约束

包括：

- 状态边界（速度、深度范围 \(z_{\min}, z_{\max}\)）
- 控制边界（加速度上限）
- 松弛变量边界（由静/动约束松弛比例换算）

问题最终被转成稀疏 QP 并由 OSQP 求解。

---

## 6. 障碍融合与预测（静态 + 动态统一约束）

### 6.1 两类障碍如何进入同一MPC

- 静态障碍：来自 occupancy map 的膨胀占据体素，经局部聚类得到包围盒。
- 动态障碍：来自 detector 速度可视化主题，解析中心与速度后形成动态体。
- 在MPC构图时，二者合并到同一个障碍集合并统一写入约束矩阵。

### 6.2 椭球安全约束与线性化

每个障碍（每个预测时刻）建模为旋转椭球外侧约束。定义安全函数：

\[
f(\mathbf{x})=
\frac{x_r^2}{a^2}+\frac{y_r^2}{b^2}+\frac{z_r^2}{c^2}
\]

要求：

\[
f(\mathbf{x})\ge 1
\]

其中 \((a,b,c)\) 来自“障碍几何尺寸/2 + 安全距离”，动态和静态分别使用 `dynamic_safety_dist` 与 `static_safety_dist`。

由于该约束非凸，代码采用上一轮解点 \(\mathbf{c}\) 做一阶线性化：

\[
\nabla f(\mathbf{c})^\top \mathbf{x} - s
\ge
1-f(\mathbf{c})+\nabla f(\mathbf{c})^\top \mathbf{c}
\]

\(s\) 为松弛变量（动态与静态分通道）。

### 6.3 动态预测与多场景择优

对动态障碍构造意图集：

\[
\mathcal{I}=\{\text{FORWARD}, \text{LEFT}, \text{RIGHT}, \text{STOP}\}
\]

并生成多组预测轨迹，调用 `makePlanWithPred()` 执行“多候选求解 + 评分择优”。

评分由一致性、绕行代价、安全性等组成，并结合意图概率加权；若预测分支失败，则回退到 `makePlan()` 保障稳定输出。

---

## 7. 参数到行为映射（调参导向）

以下按功能分组给出“增大/减小”的主要影响。

### 7.1 任务级全局覆盖参数（SIP）

| 参数组 | 增大效果 | 减小效果 |
|---|---|---|
| `max_viewpoints` | 覆盖更细、路径更长、优化时间增加 | 覆盖稀疏、优化更快 |
| `resample_iterations` | 局部可行性与路径质量通常提升 | 质量下降但更快 |
| `terrain_clearance` | 更保守、更安全 | 更贴地、风险上升 |
| `fallback_large_cost` | 不可行边惩罚更强 | 可能更易穿越薄弱可行区 |

### 7.2 在线全局轨迹参数（适配器）

| 参数组 | 增大效果 | 减小效果 |
|---|---|---|
| `waypoint_resolution`（更大=更稀） | 轨迹点更少、控制更粗 | 更密、更平滑但算量增加 |
| `path_lipb_radius` | 转角更圆滑、提前转向 | 贴近折线、角点更尖 |
| `global_pass_max_speed` | 全局通过更快 | 更稳但效率低 |

### 7.3 局部避障/MPC参数

| 参数组 | 增大效果 | 减小效果 |
|---|---|---|
| `local_trigger_distance` | 更早进入局部避障 | 更晚触发，可能来不及 |
| `dynamic_safety_dist/static_safety_dist` | 更保守，绕行更大 | 更激进，碰撞风险增 |
| `horizon` | 前瞻更长，解更稳但更慢 | 响应快但易短视 |
| `obstacle_timeout` | 容忍短时丢检 | 过期快，易“看不见障碍” |

### 7.4 感知与地图参数

| 参数组 | 增大效果 | 减小效果 |
|---|---|---|
| `occupancy_map/raycast_max_length` | 远距障碍更易入图 | 仅近距建图 |
| `occupancy_map/depth_max_value` | 远距感知范围增加 | 感知距离缩短 |
| `dynamic_velocity_threshold` | 更难判定“动态” | 更易把噪声判为动态 |

---

## 8. 实验复现与论文写作建议

### 8.1 复现实验建议流程

1. 先运行任务级覆盖规划，生成最新 `*_ros_waypoints.yaml`。
2. 将输出文件配置到在线阶段 `global_waypoint_file`。
3. 启动主仿真，再启动 detector 节点。
4. 检查关键话题与状态：
   - 动态检测输出是否持续发布
   - 占据图话题是否有数据
   - 适配器状态日志是否出现模式切换
5. 记录指标：触发距离、最小净空、重规划成功率、局部模式时长、轨迹长度与总时间。

### 8.2 论文图表建议

- 图1：系统架构图（离线规划 + 在线执行）
- 图2：地形与采样视点分布
- 图3：SIP迭代中 feasible edges 与总代价变化
- 图4：三状态模式随时间切换图
- 图5：避障场景中的全局轨迹、局部轨迹、动态障碍与占据图叠加
- 表1：关键参数与默认值
- 表2：消融实验（无预测/有预测、不同安全距离、不同触发距离）

### 8.3 论文写作建议（可直接用）

- 方法章节分成“任务级覆盖规划”和“在线局部避障MPC”两个子系统，最后给“接口耦合”小节。
- 实验章节至少做三组对照：
  - `makePlan` vs `makePlanWithPred`
  - 小安全距离 vs 大安全距离
  - 短触发距离 vs 长触发距离
- 讨论章节应明确当前系统局限：感知视场限制、预测模型简化、代价权重启发式等。

---

## 附录：关键代码锚点（仅核心入口）

1. 覆盖规划入口：`src/EROAS-main/underwater_coverage_planning/scripts/interactive_sip_coverage_planner.py`
2. 覆盖优化主函数：`src/EROAS-main/underwater_coverage_planning/scripts/sip_coverage/optimizer.py`
3. 在线适配器主循环：`src/EROAS-main/uuv_mpc_adapter/src/mpc_uuv_adapter_node.cpp`
4. MPC核心求解入口：`src/trajectory_planner/include/trajectory_planner/mpcPlanner.cpp`
5. 占据图更新核心：`src/map_manager/include/map_manager/occupancyMap.cpp`

上述 5 个锚点足以覆盖“全局生成 -> 在线跟踪 -> 局部避障 -> 地图约束”完整主链路，不需要再展开大量细碎行号。
