# Fast-LIO 前端选择和楼梯定位参数记录

本文档记录 `fast_lio_map` 和 `fast_lio` 两个前端在 M20 MID360 3D 导航中的使用结论。当前结论来自楼梯 bag 和实测反馈：两个前端在当前建图/定位参数下都能用，但实机上 `fast_lio_map` 前端有卡顿风险，因此默认切回 `fast_lio`。

## 当前结论

默认继续使用：

```text
fastlio_frontend = fast_lio
```

原因：

- 实机反馈 `fast_lio_map` 前端会卡，导航默认应优先保证前端实时性。
- 当前主导航和单独定位 launch 默认使用 `fast_lio`。
- `filter_size_surf=0.2`、`filter_size_map=0.3` 后，楼梯定位明显改善。
- `fast_lio_map` 保留为 A/B 测试和备用前端。

优先判断：

```text
之前下楼梯定位飘，主要是 Fast-LIO 前端匹配参数太粗，不是 Open3D 后端或某个前端包天然不可用。
```

## 关键参数

定位参数文件：

```text
src/m20_fastlio_nav/config/fastlio_localization_mid360.yaml
```

当前推荐：

```yaml
filter_size_surf: 0.2
filter_size_map: 0.3
acc_cov: 0.2
gyr_cov: 0.2
```

含义：

| 参数 | 作用 | 影响 |
|---|---|---|
| `filter_size_surf` | 当前帧点云降采样体素大小 | 当前保存为 `0.2`，比 `0.3/0.5` 保留更多当前帧点面约束，计算量更高 |
| `filter_size_map` | ikd-tree 局部地图降采样体素大小 | 当前保存为 `0.3`，保留楼梯/平台边缘细节，同时避免局部地图过重 |
| `acc_cov` | 加速度噪声协方差 | 当前保存为 `0.2`，对机器狗楼梯振动更放松 |
| `gyr_cov` | 角速度噪声协方差 | 当前保存为 `0.2`，对机器狗转弯和俯仰振动更放松 |

为什么当前参数比之前更适合楼梯：

- 楼梯踏步、平台边缘、墙边结构都比较细。
- `0.5m` 体素容易把这些几何约束抹掉。
- 下楼时机器狗有振动和俯仰变化，点云匹配更依赖足够密的几何约束。
- 当前帧降采样用 `0.2m`、局部地图降采样用 `0.3m`，比上一版保留更多当前帧约束。
- `acc_cov/gyr_cov=0.2` 对振动更宽容，减少前端因为 IMU 预测过硬导致的匹配失败。

不建议继续盲目调得更小。`filter_size_surf=0.1` 或 `filter_size_map=0.2` 可能继续增强约束，但会明显增加算力压力，可能造成实时性下降。除非当前参数仍然飞，再用同一个 bag 做对比测试。

## 前端切换方式

主导航默认：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh

ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true
```

等价于：

```bash
fastlio_frontend:=fast_lio
```

切到 `fast_lio_map` 做 A/B 测试：

```bash
cd /mnt/nvme/workspace/fast_lio_ws
source ./source_m20_nav.sh

ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true \
  fastlio_frontend:=fast_lio_map
```

只跑定位时：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_localization.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true \
  fastlio_frontend:=fast_lio_map
```

确认两个包都有可执行文件：

```bash
ros2 pkg executables fast_lio | grep fastlio_mapping
ros2 pkg executables fast_lio_map | grep fastlio_mapping
```

## 两个前端的关系

对 MID360 正常连续输入来说，两者核心 LIO 主流程基本一致：

```text
点云预处理
  -> IMU 去畸变
  -> 当前帧降采样
  -> ikd-tree 局部地图
  -> 点面匹配
  -> iterated EKF 更新
  -> 发布 odom/cloud
```

所以如果两个前端都使用同一套 `fastlio_localization_mid360.yaml`，`filter_size_surf/map`、IMU 噪声、外参、时间同步这些参数会比前端包名更关键。

但两个包不是完全一样：

| 项目 | `fast_lio_map` | `fast_lio` |
|---|---|---|
| 当前默认 | 否 | 是 |
| 建图链路一致性 | 建图/定位都可用，但实机反馈可能卡 | 建图/定位都可用，当前默认用于导航 |
| 核心点云-IMU匹配 | 基本一致 | 基本一致 |
| `/reset_localization` | 已补 service | 自带定位 reset 逻辑 |
| 时间跳变/断流自动 reset | 当前较弱 | 更完整 |
| 点云不足时维持 odom/TF | 当前较弱 | 更完整 |
| RoboSense M1/多子点云处理 | 有额外代码 | 无 |
| MID360 主路径 | 可用 | 可用 |

因此当前策略是：

```text
默认 fast_lio；fast_lio_map 保留为 A/B 测试和备用前端。
```

## Open3D 后端关系

这次楼梯定位改善主要来自 Fast-LIO 前端参数，不是 Open3D 后端修出来的。

Open3D 的作用是用 3D 地图修正 `map -> odom`。如果 `/Odometry_loc` 前端已经飞掉，Open3D 很难稳定救回来。判断方法：

- 如果 `/Odometry_loc` 自己大跳，问题在 Fast-LIO 前端。
- 如果 `/Odometry_loc` 平稳，但 `map -> odom` 大跳，问题更可能在 Open3D ICP 或初始位姿。

连续定位里低 fitness 不更新本来就是 Open3D 原有逻辑：

```text
fitness > threshold_fitness 时才更新 map -> odom
```

额外的 Open3D ICP 大跳变拒绝保护已经去掉，不作为当前方案的一部分。初始化阶段保留低 fitness 不写 `map -> odom` 的修正，避免一开始 ICP 不可信时污染初始变换。

## 推荐测试方法

用同一个 bag、同一张地图、同一套 `fastlio_localization_mid360.yaml`，只切 `fastlio_frontend`。

测试 `fast_lio_map`：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true \
  fastlio_frontend:=fast_lio_map
```

测试 `fast_lio`：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:="${M20_MAP_PCD}" \
  rviz:=true \
  fastlio_frontend:=fast_lio
```

播放同一个 bag：

```bash
ros2 bag play bags/stair3 --clock
```

观察：

```bash
ros2 topic echo /Odometry_loc --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

RViz 里重点看：

- `/cloud_registered_1` 是否贴合 3D 地图。
- 机器人下楼梯时 `/Odometry_loc` 是否先飞。
- `map -> base_link` 是否被 Open3D 拉跳。
- 下楼梯平台转弯时点云是否明显错位。

## 当前保留建议

短期不要删除任一前端：

- `fast_lio` 继续作为默认。
- `fast_lio_map` 保留为备用和对照。
- `fastlio_frontend` launch 参数保留，方便以后复现。

后续如果要收敛到一个方案：

1. 连续用同一组楼梯 bag 和实车下楼测试两个前端。
2. 如果两者都稳定，保持默认 `fast_lio`，优先保证实时性。
3. 如果 `fast_lio_map` 后续不再卡，再比较它和 `fast_lio` 的 reset、断流和点云不足处理逻辑。
4. 不建议直接大范围换代码；更稳的做法是把确认有效的保护逻辑补到默认前端。

## 修改后需要编译的情况

只改 YAML：

```text
不需要重新编译，重新 launch 即可。
```

改 launch 或 Python 节点：

```bash
colcon build --packages-select m20_fastlio_nav --symlink-install
source ./source_m20_nav.sh
```

改 `fast_lio` 或 `fast_lio_map` C++：

```bash
colcon build --packages-select fast_lio fast_lio_map --symlink-install
source ./source_m20_nav.sh
```
