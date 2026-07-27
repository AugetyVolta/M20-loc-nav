# Traversability Layer - 3D可通行性代价地图层

## 概述

Traversability Layer 是一个 nav2_costmap_2d 插件，用于从3D点云数据生成可通行性代价地图。相比传统2.5D高程图方案，它解决了室内环境中天花板被误判为不可通行区域的问题。

### 解决的核心问题

1. **天花板误判**：传统2.5D高程图取每个cell的最高点作为地面高度，室内环境中天花板会被当作不可通行区域
2. **平行墙体盲区**：与视线平行的墙体在raycast找梯度时可能找到相邻等高墙体，导致无cost

## 核心算法流程

```
点云输入 (PointCloud2)
    ↓
[1] 3D Voxel Grid 构建 (OpenMP并行)
    ↓
[2] Ray Tracing: 从传感器原点投射射线
    ↓ 区分地面点(hit + 上方free)和天花板点(hit + 下方free)
[3] 地面提取 (Ground Extraction)
    ↓ 计算每个cell的地面高度z
[4] 障碍物比率检测 (Obstacle Ratio)
    ↓ 检测地面上方robot_height范围内的障碍密度
    ↓ 解决平行墙体盲区问题
[5] 立方插值补全 (Interpolation)
    ↓ 填充无观测数据的地面cell
[6] 坡度与高度差计算
    ↓ 计算slope_x, slope_y, slope_magnitude, height_diff
[7] 代价计算 → Costmap2D
    LETHAL: 坡度>阈值 或 障碍比率>阈值
    INFLATED: 坡度/高度差线性加权
    FREE: 可通行区域
```

## 关键数据结构

### VoxelData - 3D体素
```cpp
struct VoxelData
{
  uint8_t hit_count = 0;   // 点云命中计数
  uint8_t pass_count = 0;  // 射线穿过计数(free space)
};
```

### GroundCell - 地面cell
```cpp
struct GroundCell
{
  float ground_z = 0.0f;          // 地面高度
  bool has_ground = false;        // 是否有地面数据
  float height_diff = 0.0f;       // 邻域最大高度差
  float slope_x = 0.0f;          // X方向坡度
  float slope_y = 0.0f;          // Y方向坡度
  float slope_magnitude = 0.0f;  // 坡度幅值
  float obstacle_ratio = 0.0f;   // 障碍物比率
};
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pointcloud_topic` | `/lio/body/cloud` | 点云输入话题 |
| `sensor_frame` | `""` (自动) | 传感器坐标系 |
| `voxel_z_resolution` | 0.1m | 体素Z轴分辨率 |
| `ground_hit_threshold` | 1 | 地面判定最小hit数 |
| `free_space_threshold` | 1 | 自由空间判定阈值 |
| `free_space_window` | 3 | 自由空间搜索窗口(Z方向cell数) |
| `robot_height` | 0.5m | 机器人高度（用于障碍比率检测） |
| `obstacle_ratio_threshold` | 0.5 | 障碍比率阈值（超过则LETHAL） |
| `max_slope_traversable` | 45.0° | 最大可通行坡度 |
| `slope_cost_start` | 15.0° | 开始产生cost的坡度 |
| `step_height_threshold` | 0.15m | 台阶高度阈值 |
| `height_cost_start` | 0.0m | 开始产生cost的高度差 |
| `slope_cost_scale` | 5.0 | 坡度cost缩放系数 |
| `height_cost_scale` | 10.0 | 高度差cost缩放系数 |
| `lethal_cost_threshold` | 254.0 | 致命cost阈值 |
| `observation_persistence` | 50 | 体素保留的点云帧数，不是秒 |
| `skip_frames` | 0 | 每 N+1 帧执行一次完整地面和代价计算 |
| `persist_cost` | false | 无新观测时是否继续使用历史 cost |
| `trust_interpolated_ground` | true | 插值地面是否写入低代价 1 |
| `transform_tolerance` | 0.3s | 按消息时间戳查询 TF 的等待时间 |
| `ground_fill_radius` | 0.5m | 机器人附近缺失地面的填充半径，0 为关闭 |
| `ground_fill_height` | 0.3m | 找不到真实地面时 base frame 到地面的高度估计 |
| `enable_perf_log` | false | 是否每 120 秒输出性能统计 |
| `cloud_buffer_size` | 5 | 点云缓冲帧数 |
| `interp_search_radius` | 3 | 插值搜索半径(cell数) |
| `min_interp_neighbors` | 2 | 插值最小邻居数 |
| `num_threads` | 0 (自动) | OpenMP线程数 |

### M20 当前实测参数

本工作空间在 `m20_fastlio_nav/config/nav2_dwb_body_plane.yaml` 中覆盖了源码默认值：

| 参数 | 当前值 | 当前行为 |
|------|--------|----------|
| `observation_persistence` | 10 | 点云约 10Hz 时，体素命中约保留 1s；按输入帧衰减 |
| `skip_frames` | 1 | 每帧累积体素，每 2 帧更新一次地面和 cost |
| `persist_cost` | false | 无新观测时不永久保留历史 cost |
| `trust_interpolated_ground` | false | 插值/填充地面不直接写成低代价 |
| `transform_tolerance` | 0.3s | 点云和 scan 均按消息时间戳等待 TF |
| `ground_fill_radius` | 0.2m | 仅补偿机器人近身地面盲区 |
| `ground_fill_height` | 0.5m | 找不到真实地面时使用的 fallback 高度 |
| `filtered_scan_min_cost` | 64 | 只保留已知且 cost 不低于 64 的 scan endpoint |

这些是当前测试组合，不等同于插件源码默认值。`persist_cost=false` 只关闭永久 cost
记忆，短时体素保留仍由 `observation_persistence` 控制。

## nav2 集成配置

在 nav2 参数文件中添加 traversability_layer 插件：

```yaml
local_costmap:
  ros__parameters:
    plugins: ["static_layer", "traversability_layer", "inflation_layer"]
    traversability_layer:
      plugin: "traversability_layer::TraversabilityLayer"
      enabled: true
      pointcloud_topic: "/lio/body/cloud"
      voxel_z_resolution: 0.1
      voxel_z_min: -1.0
      voxel_z_max: 3.0
      ground_hit_threshold: 1
      robot_height: 0.5
      obstacle_ratio_threshold: 0.5
      max_slope_traversable: 45.0
      step_height_threshold: 0.15
      num_threads: 0
```

## 性能优化

- **OpenMP并行**: 体素构建和地面提取均使用多线程，`num_threads=0`自动检测核心数
- **Thread-local缓冲**: 减少原子操作竞争，voxel写入使用线程局部缓冲后合并
- **滚动窗口同步**: voxel、ground map 和历史 cost 使用同一 costmap origin 原子移位
- **双缓冲移位**: 滚动窗口更新时复用内存，避免反复分配大数组
- **按时间戳查询TF**: 不使用最新 TF 处理旧点云，避免运动时产生位置拖影
- **Z轴自动收缩**: 上下楼后根据当前观测回收不再需要的体素高度范围
- **局部costmap缓冲**: 消除cost写入的临界区
- **数据压缩**: voxel数据从uint16_t压缩为uint8_t，使用memset快速初始化

## Filtered scan

`TraversabilityLayer` 是实时可通行过滤方案，根据当前三维点云计算的 traversability cost
过滤原始 `/scan`，不依赖离线 tomogram。参数位于
`nav2_dwb_body_plane.yaml` 的 `local_costmap.traversability_layer`：

| 参数 | M20 配置值 | 说明 |
|---|---:|---|
| `publish_filtered_scan` | `true` | 是否发布 cost 过滤结果 |
| `filtered_scan_input_topic` | `/scan` | 输入原始 LaserScan |
| `filtered_scan_topic` | `/traversability_filtered_scan` | 过滤后输出话题 |
| `filtered_scan_min_cost` | `64.0` | endpoint 所在格 cost 不低于该值才保留；未知或无地面格删除 |

scan endpoint 使用消息时间戳的 TF 转换到 local costmap 全局坐标系。TF 查询失败时发布
未过滤 scan，避免因为坐标暂时不可用而删除真实障碍。

静态 PCT tomogram 过滤已经拆到独立包 `tomogram_filter_layer`。切换时只修改同一 YAML 的
`local_costmap.plugins`，不要同时加载两个过滤插件，否则它们会同时发布
`/traversability_filtered_scan`。

当前主导航 YAML 默认使用实时 traversability 模式，插件列表是：

```yaml
plugins: ["traversability_layer", "inflation_layer"]
```

本插件本身会直接向 master costmap 写代价，因此不需要再用 `ObstacleLayer` 重复消费它
发布的 filtered scan。filtered scan 仍保留给 PCT 动态避障和 RL local path。

修改 YAML 后只需要重启主导航，不需要重新编译。修改 C++ 源码后执行：

```bash
colcon build --symlink-install --packages-select traversability_layer tomogram_filter_layer m20_fastlio_nav
```

## 编译

```bash
cd ~/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select traversability_layer
```

## 备份

原始2.5D实现已备份为：
- `src/traversability_layer.cpp.bk`
- `include/traversability_layer/traversability_layer.hpp.bk`

## 项目位置

```
LIO-SAM_MID360_ROS2_PKG/ros2/src/traversability_layer/
├── include/traversability_layer/
│   ├── traversability_layer.hpp      # 头文件
│   └── traversability_layer.hpp.bk   # 原始2.5D版本备份
├── src/
│   ├── traversability_layer.cpp      # 3D实现
│   └── traversability_layer.cpp.bk   # 原始2.5D版本备份
├── traversability_layer_plugin.xml   # 插件描述
├── CMakeLists.txt
└── package.xml
```
