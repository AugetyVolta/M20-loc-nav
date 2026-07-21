# 3D 导航动态避障问题跟踪

本文档记录当前 3D 导航中已经观察到的问题、可能原因、排查方法和后续处理顺序。目标是把 PCT 全局动态避障、DWB 局部避障、traversability filtered scan 和楼梯过滤之间的关系拆清楚，避免后续调参时把问题混在一起。

## 1. DWB 近距离不停车或继续前撞

**现象**

人或障碍物已经离机器狗很近时，狗没有立刻停住，可能继续给前进速度，甚至直接撞上。

**为什么优先级最高**

这是安全兜底问题。即使 PCT 全局路径没有及时绕开，DWB 和 local costmap 也必须负责近距离刹停。PCT 是全局规划，天然比 controller 慢，不能承担最后防撞责任。

**需要确认**

- `/local_costmap/costmap` 中人前方是否出现高代价区域。
- `traversability_layer` 是否把近距离障碍写入 local costmap。
- `inflation_layer` 是否把障碍膨胀到足够宽。
- footprint 是否覆盖真实机身，尤其前方身体、头部和腿部是否被低估。
- DWB 的 `BaseObstacle`、`PathDist`、`GoalDist`、`PathAlign` 等 critic 是否让机器人过度追路径。
- velocity smoother 或 `/NAV_CMD` adapter 是否把停车命令、小速度命令改坏。

**判断方式**

- 如果 local costmap 已经有障碍，但 DWB 仍然前进，重点查 DWB critic、速度采样、速度死区、NAV_CMD 适配。
- 如果 local costmap 没有障碍，重点查 scan、点云、traversability layer、frame 和 costmap 输入。

## 2. local costmap 对近距离障碍的感知是否可靠

**现象**

狗近距离可能不避障或不停车，说明需要先确认 local costmap 是否真的看到障碍。

**需要排查**

- RViz 中同时显示原始 `/scan`、`/traversability_filtered_scan`、local costmap 和 footprint。
- 确认障碍物进入 footprint 前方时，local costmap 是否立即出现 lethal 或 inflated cost。
- 检查 `local_costmap.update_frequency` 和 `publish_frequency` 是否正常。
- 检查 `global_frame=odom_body`、`robot_base_frame=base_link`、scan frame 是否对齐。
- 检查 traversability layer 的点云输入是否延迟或缺帧。

**后续方向**

local costmap 必须作为近距离安全层独立可靠。PCT 全局动态绕行只能改善提前绕障，不能代替 DWB 的最后刹停。

## 3. filtered scan 贴墙漏人

**现象**

人贴墙站时，PCT 更容易不把人当成障碍物处理。平地上也可能出现 filtered scan 中人体有效点变少的情况。

**当前机制**

`/traversability_filtered_scan` 现在由独立 C++ 节点直接从原始三维点云生成，只使用静态 PCT 可通行 tomogram 做 XYZ 地面匹配。slope map 不参与过滤，tomogram 未覆盖点和高于地面的障碍保留，最后才投影为 LaserScan。它不会再按 local costmap 的二维 cost 粗暴删除整条 beam。

**可能原因**

- `ground_xy_tolerance` 太大时，墙边或栏杆底部可能匹配到相邻可通行面。
- `ground_z_max_offset` 太大时，低矮障碍可能被当作地面删除。
- 定位或 tomogram 高度存在偏差时，真实地面可能无法匹配静态表面而重新进入 scan。

**后续方向**

- 不建议直接把 PCT 输入切回原始 `/scan`，否则楼梯边缘可能重新干扰全局路径。
- 优先观察日志中的 `ground_removed`，再调 tomogram 的 XYZ 容差。
- 保持 `keep_points_without_ground=true`，让 tomogram 未覆盖区域中的真实障碍继续保留。

## 4. PCT 动态点被静态墙吞掉

**现象**

即使 filtered scan 里还有人体点，贴墙时 PCT 也可能没有把它写入动态障碍层。

**当前机制**

PCT 有静态障碍跳过逻辑：

```text
global_path_perception_skip_static_obstacles = true
```

这个逻辑本来是为了避免静态 tomogram 里的墙、楼梯结构被重复写入动态层，否则全局路径会被静态结构反复膨胀干扰。

**可能原因**

人贴墙时，scan endpoint 可能落在静态 tomogram 里的墙格上。PCT 会认为这是静态障碍，于是跳过该动态点。结果是动态硬核心没有写进去，全局路径自然不会绕人。

**后续方向**

- 不能简单关闭 `skip_static_obstacles`，否则墙和楼梯边缘可能重新进入动态层。
- 可以只对来自 filtered scan 的有限 hit 做更温和的 static skip。
- 可以在 static skip 前做邻域搜索：如果 endpoint 落在静态墙格，但附近有可写入的非静态格，把动态点写到最近可写入格。
- 可以给 scan mark 增加小半径写入，而不是只写 endpoint 单格。

## 5. PCT 全局动态避障响应慢

**现象**

人站到前方后，全局路径不像 Nav2 2D global planner 那样迅速变化。路径有时过一会儿才绕，有时又恢复直线。

**原因**

PCT 是 3D 全局规划，不是 Nav2 2D global planner。当前流程更重：

1. 更新动态 perception layer。
2. 在 3D tomogram 上做 A*。
3. 做路径后端平滑/优化。
4. 发布 `/pct_path`。
5. pure pursuit、RL local path、DWB 再跟踪这条路径。

`pct_replan_interval` 只是 timer 触发周期，不代表一定能按这个周期产出路径。如果一次 PCT 规划耗时 700ms，把周期设成 0.5s 也不能保证 0.5s 出一条新路径。

**需要观察的日志**

```text
Published /pct_path with ... poses in XXX ms
```

**判断方式**

- 如果规划耗时基本小于 200ms，可以考虑更高频重规划。
- 如果经常 400ms 到 800ms，`0.5s` 周期收益有限。
- 如果超过 1s，应优先优化 PCT 计算或降低规划频率。
- 规划慢时，更应依赖 DWB/local costmap 做近距离安全避障。

## 6. PCT 规划可能阻塞动态感知 callback

**现象**

动态障碍更新有时延，scan 到了但 PCT 动态层不一定立即更新。

**当前机制**

`pct_planner_node.py` 当前使用普通单线程 spin：

```python
rclpy.spin(node)
```

而规划函数中直接调用：

```python
traj_3d = self.planner.plan(self.start_pos, self.goal_pos)
```

如果 `planner.plan()` 耗时较长，同一个 executor 中的 scan callback 也会被阻塞。

**影响**

- filtered scan 到了，但 PCT 没及时处理。
- 动态障碍 source grid 更新滞后。
- raytrace clearing 滞后。
- 全局路径相对实际障碍慢一拍。

**后续方向**

- 使用 `MultiThreadedExecutor`。
- 给 scan callback 和 timer callback 分 callback group。
- 更进一步，把规划放到单独 worker thread。
- 注意不能让多个 `planner.plan()` 并发访问同一个 planner 对象。
- 如果规划还没结束，新的规划请求只记录 pending 状态，等当前规划完成后立即再跑一次。

## 7. 全局路径对人只是轻微侧偏，不明显绕开

**现象**

人站在机器人前方、全局路径上时，PCT 路径不是像 Nav2 2D global planner 一样明显绕开，而是很平滑地小幅偏移。

**可能原因**

- 动态障碍在 PCT 中不是完整 2D costmap lethal block，而是动态 source 点和 inflation。
- filtered scan 点少，动态障碍区域不连续。
- 动态硬核心没有真正写入，A* 认为轻微绕一点就够。
- A* 已经绕开，但后端平滑又把路径拉回来了。
- PCT 本身倾向生成平滑 3D 路径，会减少曲率和路径长度。

**后续方向**

- 可视化动态 perception layer。
- 增加 `/pct_astar_path` debug topic。
- 对比 A* 原始路径和最终 `/pct_path`。
- 如果 A* 没绕，问题在动态代价、mark 或 hard core。
- 如果 A* 绕了但最终没绕，问题在平滑/优化。

## 8. PCT 后端平滑可能削弱动态避障

**现象**

全局路径看起来很平滑地绕一点，没有明显避开障碍。可能 A* 已经绕开了人，但后端优化把路径重新拉直。

**可能机制**

PCT 不是直接发布 A* 路径，而是 A* 后还有平滑/优化。如果动态障碍只在 A* 阶段强制避开，但后端平滑阶段只是看到软代价，那么可能出现：

- A* 原始路径绕开人。
- 平滑器为了减少曲率和路径长度，把路径往直线拉。
- 最终 `/pct_path` 又贴近人，甚至穿回动态障碍附近。

**后续方向**

- 发布 `/pct_astar_path`，保留 A* 原始路径。
- 让 `/pct_path` 和 `/pct_astar_path` 同时在 RViz 中显示。
- 如果确认平滑拉坏：
  - 降低动态障碍附近的平滑权重；
  - 增强后端 obstacle factor；
  - 对 `inscribed_radius` 内动态障碍设置 hard 或极高 penalty；
  - 动态障碍存在时限制平滑器偏离 A* 路径过多。

## 9. 动态硬核心是否真正写入 PCT

**当前改动**

已经加入：

```text
global_path_perception_inscribed_radius = 0.35
```

理论上该半径内的动态障碍格子在 A* 中不可通行。

**风险**

硬核心能否生效，取决于前面有没有 mark 到动态点。如果 filtered scan 把人删了，或者 static skip 把人跳过了，那么硬核心根本不会生成。

**需要观察**

- `mark_cells` 数量。
- `active_cells` 数量。
- `clear_cells` 是否过多。
- 人站到前方时 dynamic perception layer 是否出现连续区域。
- 贴墙时 active cells 是否明显减少。

当前日志中已有：

```text
Updated C++ PCT global path perception:
active_cells=...
changed_cells=...
clear_cells=...
mark_cells=...
```

后续可以进一步增加 debug topic，把 dynamic perception layer 可视化出来。

## 10. 动态障碍清除/保留是否过于抖动

**现象**

人站在前方时，路径有时绕，有时又恢复直线；楼梯或平台附近路径左右晃。

**可能原因**

- filtered scan 本身抖动。
- raytrace clearing 把动态 source 清掉太快。
- `persistence` 虽然保留一段时间，但主要清除机制是 raytrace。
- 一条自由射线可能清掉局部动态障碍。
- PCT 每次重规划看到的动态层不一致。
- 贴墙/楼梯边缘时 scan endpoint 跳动更明显。

**后续方向**

- 统计每次 mark/clear 数量。
- 限制 raytrace 对刚刚 mark 的点立即清除。
- 增加动态 source 的最短保留时间。
- 对 mark 点做时间窗口累积，而不是每帧完全依赖当前 scan。

## 11. PCT 全局绕行和 DWB 局部避障职责边界不清

**问题**

当前系统有时过度依赖 PCT 全局路径绕动态人，但近距离安全应由 DWB 和 local costmap 负责。

**合理分工**

- PCT：负责较远处的路径形状变化，例如人提前挡在走廊中间，全局路径提前绕。
- DWB：负责近距离避障、减速、刹停、防撞。
- local costmap：必须实时反映近距离障碍。
- filtered scan：服务 PCT 动态全局层，但不能影响 DWB 的安全底线。

PCT 算法周期和规划时延天然比 controller 慢，因此不能承担所有避障责任。

## 12. PCT 和 Nav2 2D global planner 行为差异

**Nav2 2D global planner 的机制**

- obstacle layer 把障碍写成 lethal。
- inflation layer 生成连续膨胀代价。
- global planner 在连续 2D costmap 上搜索。
- local controller 同时看同一套或相似 costmap。

**当前 PCT 机制**

- 静态 3D tomogram。
- 动态 scan 点叠加到 perception layer。
- 3D A* 搜索。
- 后端平滑。
- 发布 `/pct_path` 给后续控制链路。

因此 PCT 不会天然等价于 Nav2 global planner。要接近 Nav2，需要让 PCT 的动态层更像 costmap：

- 动态障碍区域连续。
- 核心 hard block。
- 外圈 inflation。
- clearing 稳定。
- 平滑不能破坏绕障。
- 起点随 `base_link` 更新。

## 13. 楼梯过滤和动态避障存在冲突

**问题**

filtered scan 是为了解决楼梯问题引入的。当前版本使用三维点相对 tomogram 地面的真实高度，不再用二维 cost 阈值猜测地面，但匹配容差仍需要兼顾定位误差和低矮障碍。

**容易冲突的场景**

- 人贴墙。
- 人或栏杆紧贴可通行 tomogram cell 边界。
- 定位 Z 偏差导致地面与 tomogram 对不齐。
- 低矮障碍高度落入地面 Z 容差。

**后续原则**

- 不能直接用原始 `/scan` 代替 filtered scan，否则楼梯边缘可能重新导致 PCT 路径失败或乱绕。
- `ground_z_max_offset` 不应盲目调大，否则低矮障碍会漏检。
- 未匹配到 tomogram 地面的点应继续保留，近距离安全仍由 DWB local costmap 独立兜底。

## 14. PCT 重规划周期是否应该调快

**结论**

不能只靠把 `pct_replan_interval` 改小来解决响应慢。

**判断标准**

- 如果 `Published /pct_path ... in XXX ms` 基本小于 200ms，可以考虑 `0.5s`。
- 如果经常 400ms 到 800ms，`0.5s` 边际收益有限。
- 如果超过 1s，应优先优化 PCT 计算或减少规划频率。
- 规划慢时，更应该依赖 DWB/local costmap 做近距离防撞。

## 15. 是否需要把 PCT 规划线程化

**需要条件**

如果确认 PCT 规划阻塞 scan callback，就需要改节点结构。

**目标结构**

- scan callback 只更新动态 perception layer。
- timer 触发规划请求。
- 规划任务在单独 worker thread 跑。
- 当前规划未结束时，新请求只记录 pending 状态。
- 当前规划完成后，如果 pending 为真，立即再跑一次。
- 避免多个 `planner.plan()` 并发访问同一个 planner 对象。

**预期收益**

线程化不能让单次 PCT 规划变快，但可以降低 scan 动态层更新时延，避免感知 callback 被规划计算阻塞。

## 16. 是否需要增加 debug 可视化

**结论**

需要。否则很难判断问题到底发生在哪一层。

**建议新增或同时显示的 topic**

- `/scan`：原始 LaserScan。
- `/traversability_filtered_scan`：过滤后给 PCT 的 LaserScan。
- `/local_costmap/costmap`：DWB 实际使用的 local costmap。
- `/pct_dynamic_obstacles`：PCT 动态 mark 点。
- `/pct_dynamic_inflation`：PCT 动态膨胀区域。
- `/pct_astar_path`：A* 原始路径。
- `/pct_path`：最终平滑路径。

**要回答的问题**

- 原始 scan 是否有人体点？
- filtered scan 是否保留了人体点？
- PCT 是否 mark 了动态点？
- 动态硬核心是否生成？
- A* 是否绕开？
- 平滑是否把绕障路径拉坏？
- DWB 是否在近距离停下？

## 建议后续解决顺序

1. 先解决 DWB/local costmap 近距离不停车问题，确保不撞。
2. 对比 `/scan` 和 `/traversability_filtered_scan`，确认贴墙人点是否被过滤。
3. 修改 filtered scan：加 endpoint 邻域容错和必要的近距离保护。
4. 修改 PCT static skip：避免贴墙人点被静态墙完全吞掉。
5. 加 `/pct_astar_path`，对比 A* 和最终平滑路径。
6. 如果确认平滑拉坏路径，再改后端平滑/obstacle factor。
7. 最后再评估 `pct_replan_interval`、多线程 executor、规划线程化。
