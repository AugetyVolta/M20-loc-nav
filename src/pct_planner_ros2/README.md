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
resolution: 0.2
ground_h: 0.0
slice_dh: 1.0
kernel_size: 5
interval_min: 0.45
interval_free: 0.6
slope_max: 0.6
step_max: 0.4
standable_ratio: 0.1
cost_barrier: 50.0
safe_margin: 0.25
inflation: 0.1
```

生成 tomogram：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh
ros2 launch pct_planner_ros2 m20_tomography_rviz.launch.py \
  pct_root:="${PCT_PLANNER_ROOT}"
```

一次运行会同时生成两个文件，位置在传入的 `pct_root` 下：

```bash
${PCT_PLANNER_ROOT}/rsc/tomogram/m20_3d_map.pickle
${PCT_PLANNER_ROOT}/rsc/tomogram/m20_3d_map.surface.pcd
```

`.pickle` 保存完整 tomogram，供 PCT planner 使用；`.surface.pcd` 保存 `XYZ + cost`，
供 `tomogram_filter_layer` 直接加载。过滤插件不再依赖 PCT planner 在线发布表面。
已有 pickle 可以直接补生成 PCD：

```bash
ros2 run pct_planner_ros2 pct_export_tomogram_surface m20_3d_map
```

主导航 launch 不会把 tomogram 名称和阈值自动写入 Nav2 参数。修改
`output_tomogram_name` 或规划时的 `pct_tomogram_file` 后，还要手动修改
`src/m20_fastlio_nav/config/nav2_dwb_body_plane.yaml`：

```yaml
tomogram_surface_file: "/absolute/path/to/m20_3d_map.surface.pcd"
traversable_cost_max: 45.0  # 与 pct_a_star_cost_threshold 保持一致
```

注意：`ros2 launch` 从 `install/pct_planner_ros2/share/...` 查找 YAML。当前工作空间使用
`colcon build --symlink-install`，该文件最终链接到 `src/pct_planner_ros2/config/tomography.yaml`，
所以修改参数后重启 tomography 即可，不需要重新编译。只有重新建立了非 symlink 安装空间时，
才需要再次执行 `colcon build`。

## 当前稳定的规划参数

`path_ground_offset` 是优化路径相对 tomogram 地形表面的目标高度，单位为米。当前值
`0.10` 让路径贴近楼梯和平面；坐标转换不会再额外叠加旧版固定的 `0.5m` 高度。

当前推荐直接启动：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py
```

等价于：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  tomogram_file:=m20_3d_map \
  tomogram_visual_cost_max:=45 \
  step_cost_weight:=1.0 \
  path_ground_offset:=0.10
```

当前默认规划参数：

```text
tomogram_file = m20_3d_map
a_star_cost_threshold = 45.0
step_cost_weight = 1.0
layer_match_height_tolerance = 1.2
path_ground_offset = 0.10
tomogram_visual_cost_max = 45.0
```

全局路径感知更新默认在单独调试 launch 中关闭，在 `m20_fastlio_nav.launch.py` 主导航中默认开启。它不会修改静态 tomogram。C++ core 按 Nav2 的 obstacle/inflation 语义维护独立动态层：

- PCT 只订阅原始 `/scan`。同一帧先按 beam raytrace 写入自由空间，再把有限 endpoint 写成 `LETHAL_OBSTACLE=254`。
- scan 使用消息时间戳对应的 TF，不使用规划定时器中的旧机器人位置。
- 障碍源格为 `254`，内切半径内为 `253`，外圈使用 Nav2 指数公式衰减。A* 无条件拒绝 `253/254`，不会再被楼梯 `ele` 分支绕过。
- 同一物理地面的等高 tomogram layer 在 `0.75m` 容差内一起 mark/clear，避免 A* 从重叠 layer 穿过动态障碍；不会标记不同楼层。
- 不再按机器人半径或 footprint 额外过滤、清空动态障碍点，避免误删贴近机身的真实回波。
- 动态源、膨胀格和过期检测只遍历当前活动集合，不再扫描完整 3D tomogram。

动态层同时保留两套量纲。`perception_nav2_cost` 使用 `0..254`，其中 `254` 是障碍源、`253` 是硬核心；A* 在展开邻居前直接拒绝 `>=253`。其余软代价按
`perception_cost = perception_peak_cost * perception_nav2_cost / 254` 映射回 PCT 量纲，再与静态 tomogram cost 取最大值。`global_path_perception_cost=-1` 时，峰值自动取 `a_star_cost_threshold+5`；主导航阈值为 `45`，所以动态峰值为 `50`。

`global_path_perception_min_range=0.15` 与当前 `pointcloud_to_laserscan.range_min` 对齐，只负责过滤雷达无法可靠测量的近距数据，不是机器人碰撞半径。PCT 不再额外剔除机器人 footprint 或某个自清除半径内的有限回波；`global_path_perception_inscribed_radius=0.45` 只负责在每个有效障碍点周围建立 A* 不可进入的圆形硬核心。不要再把 `min_range` 调到 `0.45`，否则障碍突然靠近狗头时会形成真实的感知盲区。

规划器保留一条不含动态障碍的静态参考路径。定周期更新时，以 `local_replan_forward_distance` 选择基础局部段，并额外带入 `1.0m` 静态参考重叠段共同优化。优化器使用重叠段末端的参考切线作为终点方向，并把终点位置精确放回参考路径后再拼接静态后缀，从而避免接缝尖角。A* 左右搜索范围不受参考路径走廊限制；动态观测也不会按参考路径走廊二次裁剪。

单独调试时启用：

```bash
ros2 launch pct_planner_ros2 m20_pct_rviz.launch.py \
  global_path_perception_enabled:=true \
  global_path_perception_scan_topic:=/scan
```

常用参数：

```text
global_path_perception_width = 8.0                    # 机器人中心前后各约 4m
global_path_perception_height = 8.0                   # 机器人中心左右各约 4m
global_path_perception_update_interval = 0.2          # 动态层最多按 5Hz 吸收 /scan
global_path_perception_min_range = 0.15               # 对齐 /scan.range_min；不做额外自车范围过滤
global_path_perception_inflation_radius = 1.0        # 动态代价覆盖到障碍点外 1.0m
global_path_perception_inscribed_radius = 0.45       # 253 硬障碍区半径
global_path_perception_cost_scaling_factor = 5.0     # M20 主导航推荐值，对齐 Nav2 inflation
global_path_perception_persistence = 5.0             # M20 主导航推荐值；兜底过期时间，主要靠 raytrace clearing 清除
global_path_perception_raytrace_enabled = true       # 用 LaserScan 自由射线清除动态层
global_path_perception_raytrace_max_range = 0.0      # 0.0 表示自动使用感知窗口和 scan range_max 的较小值
global_path_perception_raytrace_max_rays = 360       # 每次最多处理的清除射线数
global_path_perception_height_tolerance = 0.75       # 同一物理表面的重叠 layer 高度容差
global_path_perception_mark_all_layers = false       # 不跨不同楼层写入动态障碍
local_replan_enabled = true                          # 修复当前位置到前方局部段并平滑衔接
local_replan_forward_distance = 10.0                 # 基础前视；内部再优化 1.0m 重叠段
```

完整 M20 导航 launch 会覆盖为 `global_path_perception_width=8.0`、`global_path_perception_height=8.0`、scan topic `/scan`、`min_range=0.15`、`inflation_radius=1.0`、`inscribed_radius=0.45`、`cost_scaling_factor=5.0`、`persistence=5.0`。该窗口以机器人为中心覆盖前后、左右各约 `4m`。局部修复使用 `10.0m` 基础前视和内部 `1.0m` 衔接重叠段，A* 不限制左右搜索范围。楼梯模式关闭 PCT 动态层，因此原始 scan 中的台阶不会写入 PCT。

动态层变化只更新 C++ 临时代价层，不单独立即触发重规划。动态层默认按 `global_path_perception_update_interval=0.2s`（5 Hz）吸收 `/scan`，主导航默认 `always_replan=true`，所以局部前缀按 `replan_interval=1.0s`（1 Hz）定周期刷新；最终目标未变时不会重新规划整张地图。动态观测仍由 `global_path_perception_width/height` 限制在机器人近场，但不再按参考路径走廊二次裁剪。LaserScan 默认写当前物理表面对应的等高 PCT layers。楼梯状态根据静态参考路径预判，因此动态局部规划失败也不会阻止进入楼梯模式；楼梯段仍关闭 PCT 动态避障。

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

默认会按周期重新规划。

当前参数在：

```bash
src/pct_planner_ros2/config/pct_planner.yaml
```

关键项：

```yaml
auto_plan: true
replan_interval: 1.0
always_replan: true
```

含义：

```text
每 1 秒规划并发布一次 /pct_path。
动态感知层只更新 PCT 临时代价层，不单独触发即时重规划。
如果显式把 always_replan 改成 false，只在起点或终点实际变化后重规划。
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
