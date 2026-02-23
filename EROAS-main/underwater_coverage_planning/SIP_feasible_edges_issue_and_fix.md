# SIP 全局覆盖 `feasible_edges=194/201` 问题与修复说明

## 1. 问题现象

规划日志中长期出现：

```text
Initial viewpoints: 201
Iteration k/6: solver=lkh-atsp, tour_cost=600xxxx.xx, feasible_edges=194/201
```

表现为：

- 每轮都有约 7 条边不可行。
- 总代价被 `fallback_large_cost`（`1e6`）拉高到 `600xxxx` 量级。
- 轨迹中会出现个别“穿过障碍/地形”的连线段。

## 2. 根因分析

这不是 LKH 本身的问题，根因在**视点采样阶段**：

- 旧逻辑在 `_make_viewpoint(...)` 中没有检查候选点是否落在地形高度网格内。
- 因此会生成少量 XY 越界视点（例如 `x < grid_x_min` 或 `y > grid_y_max`）。
- 这些越界点在连线可行性检查 `edge_min_clearance(...)` 时会被判定为不可行，导致这些点对其他点的入边/出边为 0（孤立）。
- 巡回必须包含所有点，LKH 只能用大罚值边把孤立点“硬连”进去，于是出现 `194/201` 和超大 `tour_cost`。

## 3. 代码修复

### 3.1 过滤越界视点（核心修复）

文件：`scripts/sip_coverage/viewpoints.py`

- 在 `_make_viewpoint(...)` 中新增：
  - 候选点生成后先执行 `terrain.is_inside_xy(...)`；
  - 若不在网格内，直接丢弃该候选。
- 当一个面在当前 standoff/jitter 下没有任何合法候选时：
  - 返回 `None`（不再生成“兜底但可能非法”的视点）。
- 在两处调用链中处理 `None`：
  - `sample_initial_viewpoints(...)`：跳过该视点；
  - `generate_resample_candidates(...)`：只保留非 `None` 候选。

这一步直接消除了“零入度/零出度”孤立点。

### 3.2 去掉“人为卡在 201”

文件：`config/sip_uuv_planner.yaml`

- `max_viewpoints` 从 `220` 调整为 `240`，避免长期固定在 `201` 点。

## 4. 修复后验证结果

同一套配置与地图下实测：

- 修复前：`Initial viewpoints: 201`，`feasible_edges=194/201`
- 修复后：`Initial viewpoints: 217`，每轮均为 `feasible_edges=217/217`

示例（修复后）：

```text
Iteration 1/6: ... feasible_edges=217/217
Iteration 2/6: ... feasible_edges=217/217
...
Iteration 6/6: ... feasible_edges=217/217
```

## 5. 结论

- 本问题本质是**采样边界合法性**问题，不是求解器（LKH）问题。
- 只要视点采样阶段保证点在地形网格内，`feasible_edges` 会恢复到 `N/N`，大罚值边和“穿障碍连线”会显著减少或消失。

## 6. 复现/检查命令

```bash
python3 src/EROAS-main/underwater_coverage_planning/scripts/interactive_sip_coverage_planner.py \
  --config src/EROAS-main/underwater_coverage_planning/config/sip_uuv_planner.yaml \
  --no-open
```

观察日志中的 `Initial viewpoints` 与 `feasible_edges` 即可快速判断是否回归。

