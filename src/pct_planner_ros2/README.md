# PCT Planner ROS2 使用说明

这个包是把当前调好的 PCT planner 内置到 `fast_lio_ws` 的 ROS2 Humble 适配工程，路径为：

```bash
/mnt/nvme/workspace/fast_lio_ws/src/pct_planner_ros2
```

当前使用的输入点云是：

```bash
/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_3d_map.pcd
```

Open3D 和 CuPy 使用这个虚拟环境：

```bash
/home/orin/venv/m20_nav_cupy
```

## 目录说明

```text
PCT_planner/        当前调好的 PCT planner 代码拷贝，含 tomography 和 planner 核心
config/             当前使用的 YAML 和 RViz 配置
launch/             生成 tomogram、启动 planner、打开 RViz 的 launch 文件
pct_planner_ros2/   ROS2 Python 节点
scripts/            环境加载和 PCT core 编译脚本
```

ROS2 Python 节点在：

```text
pct_planner_ros2/
```

主要文件作用：

```text
pct_tomography_node.py   PCD 转 PCT tomogram
pct_planner_node.py      读取 tomogram 并规划 /pct_path
pct_map_viz_node.py      在 RViz 发布原始 PCD 和 tomogram 可视化
pct_paths.py             PCT 路径、PYTHONPATH、LD_LIBRARY_PATH 辅助函数
check_env.py             环境检查命令 pct_check_env
```

## 环境加载

每次打开新终端，先执行：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh
```

根目录 `source_m20_nav.sh` 会加载：

```text
ROS2 Humble
install/setup.bash
PCT planner Python 路径
PCT C++/pybind 动态库路径
/home/orin/venv/m20_nav_cupy
```

## 编译

修改 launch、YAML、Python 节点或 README 后，执行：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh
colcon build --packages-select pct_planner_ros2
source ./source_m20_nav.sh
```

如果改了 PCT C++/pybind 核心，再执行：

```bash
./src/pct_planner_ros2/scripts/build_pct_core.sh
```

检查环境：

```bash
ros2 run pct_planner_ros2 pct_check_env
```

## 当前稳定的 Tomogram 参数

参数文件：

```bash
src/pct_planner_ros2/config/tomography.yaml
```

当前保存的稳定版本：

```yaml
output_tomogram_name: "m20_3d_map"
resolution: 0.15
ground_h: -1.0
slice_dh: 0.5
kernel_size: 5
interval_min: 0.45
interval_free: 0.3
slope_max: 0.6
step_max: 0.4
standable_ratio: 0.1
cost_barrier: 50.0
safe_margin: 0.15
inflation: 0.05
```

生成 tomogram：

```bash
ros2 launch pct_planner_ros2 m20_tomography_rviz.launch.py
```

输出文件：

```bash
PCT_planner/rsc/tomogram/m20_3d_map.pickle
```

注意：`ros2 launch` 实际读取的是 `install/pct_planner_ros2/share/...` 下的 YAML。修改 `src/pct_planner_ros2/config/tomography.yaml` 后，必须重新 `colcon build`，否则 launch 仍然会用旧参数。

## 当前稳定的规划参数

当前推荐直接启动：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py
```

等价于：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  tomogram_file:=m20_3d_map \
  tomogram_visual_cost_max:=45 \
  step_cost_weight:=1.0
```

当前默认规划参数：

```text
tomogram_file = m20_3d_map
a_star_cost_threshold = 45.0
step_cost_weight = 1.0
layer_match_height_tolerance = 1.2
tomogram_visual_cost_max = 45.0
```

全局路径感知更新默认在单独调试 launch 中关闭，在 `m20_fastlio_nav.launch.py` 主导航中默认开启。它不会在线重建完整 tomogram，也不会修改静态 tomogram；C++ core 会单独维护一层带时间戳的临时代价，A* 和后端 DenseElevationMap 查询时使用 `max(static_cost, perception_cost)`。主导航默认输入是 `traversability_layer` 发布的 `/traversability_filtered_scan`，也就是原始 `/scan` 经过可通行层过滤后的障碍 scan：只有明确高代价/不可通行的 endpoint 保留，可通行、未知、无地面 endpoint 置为 `inf`。临时代价使用和 Nav2 local costmap inflation 类似的距离衰减，无新观测超时或机器人经过清理半径后会自动恢复为静态 tomogram cost。动态层变化只更新代价层，不再单独触发即时重规划；主导航用 `replan_interval` 定周期发布新路径。

单独调试时启用：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  global_path_perception_enabled:=true \
  global_path_perception_scan_topic:=/traversability_filtered_scan
```

常用参数：

```text
global_path_perception_width = 4.0                    # 单独 PCT RViz 调试 launch 默认值
global_path_perception_height = 4.0                   # 单独 PCT RViz 调试 launch 默认值
global_path_perception_inflation_radius = 0.45       # M20 主导航推荐值，减少楼梯转角大绕行
global_path_perception_cost_scaling_factor = 8.0     # M20 主导航推荐值，让动态代价更快衰减
global_path_perception_persistence = 0.6             # M20 主导航推荐值，减少旧障碍残留
global_path_perception_path_corridor_radius = 0.4    # 只让最新路径附近的动态点影响全局路径
global_path_perception_skip_static_obstacles = true  # 静态 tomogram 高代价点不重复写入动态层
```

完整 M20 导航 launch 会覆盖为 `global_path_perception_width=6.0`、`global_path_perception_height=6.0`、`global_path_perception_scan_topic=/traversability_filtered_scan`、`global_path_perception_inflation_radius=0.45`、`global_path_perception_cost_scaling_factor=8.0`、`global_path_perception_persistence=0.6`，并默认开启全局路径感知。其余滤波、层匹配、机器人清除半径和感知峰值 cost 使用节点默认值；峰值 cost 默认自动取 `a_star_cost_threshold + 5`。

动态层变化只更新 C++ 临时代价层，不会单独立即触发重规划。主导航默认 `always_replan=true`，所以 `/pct_path` 按 `replan_interval=1.0s` 定周期刷新。`global_path_perception_path_corridor_radius=0.4` 使用最新 `/pct_path` 做路径中心线，只让路径附近的动态点影响全局路径；`global_path_perception_skip_static_obstacles=true` 会忽略静态 tomogram 中已经高于 `a_star_cost_threshold` 的墙体/结构点。

因为全局路径感知更新改在 PCT C++/pybind core 内，修改后需要重新构建 core：

```bash
./src/pct_planner_ros2/scripts/build_pct_core.sh
colcon build --packages-select pct_planner_ros2
```

RViz 中使用 `Interact` 工具拖动：

```text
start_pos
end_pos
```

规划结果发布到：

```bash
/pct_path
```

显示完整 tomogram，包括红色高代价区域：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  tomogram_visual_cost_max:=50
```

`tomogram_visual_cost_max` 只影响 RViz 显示，不会修改真实规划地图。

## 是否会定期重新规划

会定期检查，但不会无条件一直重新规划。

当前参数在：

```bash
src/pct_planner_ros2/config/pct_planner.yaml
```

关键项：

```yaml
auto_plan: true
replan_interval: 1.0
position_epsilon: 0.01
```

含义：

```text
每 1 秒检查一次。
只有起点/终点变化超过 0.01m，或全局路径感知更新改变时，才重新规划。
```

如果 `start_source:=fixed`，拖动 `start_pos/end_pos` 后会重新规划。

如果 `start_source:=odom`，机器人 odom 更新起点，机器人移动后会触发重新规划：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  start_source:=odom \
  odom_topic:=/odom_body
```

## Launch 文件说明

### m20_tomography_rviz.launch.py

用途：当前 m20 地图生成 tomogram，并打开 RViz。

```bash
ros2 launch pct_planner_ros2 m20_tomography_rviz.launch.py
```

建图参数来自：

```bash
src/pct_planner_ros2/config/tomography.yaml
```

### m20_pct_rviz.launch.py

用途：当前 m20 交互式规划调试。它会启动：

```text
pct_planner_node
pct_map_viz_node
rviz2
```

会发布：

```text
/global_points   原始 PCD 可视化
/tomogram        PCT tomogram 可视化
/pct_path        规划路径
```

### tomography.launch.py

用途：通用 tomogram 生成，不打开 RViz。

它现在和 m20 版本一样，只从 `tomography.yaml` 读取建图参数，不在 launch 里覆盖参数。

```bash
ros2 launch pct_planner_ros2 tomography.launch.py
```

### pct_planner.launch.py

用途：通用 planner-only，不打开 RViz，不发布 PCD/tomogram 可视化。

默认参数已经和当前 m20 稳定规划参数对齐：

```text
tomogram_file = m20_3d_map
a_star_cost_threshold = 45.0
step_cost_weight = 1.0
layer_match_height_tolerance = 1.2
```

后续接入机器人系统，只需要规划节点和 `/pct_path` 时可以用：

```bash
ros2 launch pct_planner_ros2 pct_planner.launch.py
```

## 常见问题

### RViz 里看起来紫色很多，是否代表全部可通行？

不一定。颜色来自 `PointCloud2` 的 intensity 映射，真实是否可通行要看 cost。

规划失败时终端会打印：

```text
plan_info={... 'cost': ...}
```

判断：

```text
cost = 50        tomogram 中已经是障碍
cost < 45        当前默认 planner 可以搜索
45 < cost < 50   高代价区域，默认会尽量避开或拒绝
```

### 路径太贴墙怎么办？

先调 planner，不要马上重新生成 tomogram：

```bash
step_cost_weight:=1.0
```

如果仍然贴墙，再考虑重新生成 tomogram，并适当增大：

```yaml
safe_margin
inflation
```

### 门洞或楼梯看起来可走但规划不过怎么办？

先用完整显示排查：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  tomogram_visual_cost_max:=50
```

然后看终端 `plan_info`。

如果目标点 cost 是 50，说明该点被 tomogram 判成障碍，需要重新调 `tomography.yaml`。

如果同一 XY 有可走层但 marker 高度差太大，`layer_match_height_tolerance` 会影响层匹配。当前默认是：

```text
layer_match_height_tolerance = 1.2
```

## 上传 GitHub 注意事项

不要提交这些生成文件：

```text
build/
install/
log/
PCT_planner/rsc/tomogram/*.pickle
PCT_planner/rsc/pcd/*.pcd
*.pyc
__pycache__/
```

这些已写入根目录 `.gitignore`。

如果需要共享 `m20_3d_map.pickle`，建议放 GitHub Release、网盘或 Git LFS，不建议直接提交到普通 Git 历史。
