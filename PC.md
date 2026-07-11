```text
建好的 3D 地图
  + 同一个 bag 回放的雷达数据
  + 地图定位得到当前位姿
  + PCT 全局路径
  + Pure Pursuit 局部目标
  + RL/PRIEST 局部路径
  = RViz 实时显示 /local_path
```

注意：这是**影子测试**。机器狗的位置仍按 bag 中录制的轨迹移动，规划器输出不会改变 bag 的运动。

## 0. 先确认必要文件

在项目根目录执行：

```bash
cd ~/xlab/M20-loc-nav

ls -lh maps/fastlio/global_map.pcd
ls -lh maps/fastlio/m20_3d_map.pcd
ls -lh src/pct_planner_ros2/PCT_planner/rsc/tomogram/m20_3d_map.pickle
```

必须至少存在：

```text
maps/fastlio/m20_3d_map.pcd
src/pct_planner_ros2/PCT_planner/rsc/tomogram/m20_3d_map.pickle
```

如果第二个文件不存在，说明目前只是建好了点云地图，还没有生成 PCT 使用的层析地图，暂时不能产生 `/pct_path`。

## 1. 终端一：启动完整导航链路

```bash
cd ~/xlab/M20-loc-nav

export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=/tmp/m20_ros_logs
export MPLCONFIGDIR=/tmp/matplotlib
export XLA_PYTHON_CLIENT_PREALLOCATE=false

mkdir -p "$ROS_LOG_DIR" "$MPLCONFIGDIR"

source /opt/ros/jazzy/setup.bash
source ~/xlab/liv_ws/install/setup.bash
source install/setup.bash
```

然后启动：

```bash
ros2 launch m20_fastlio_nav m20_fastlio_nav.launch.py \
  use_sim_time:=true \
  map_pcd:=$PWD/maps/fastlio/m20_3d_map.pcd \
  rviz:=true \
  rl_python_executable:=$PWD/.venv/m20_nav_jazzy/bin/python \
  pct_venv_site:=$PWD/.venv/m20_nav_jazzy/lib/python3.12/site-packages \
  pct_tomogram_file:=m20_3d_map \
  start_adapter:=false
```

这里几个参数很重要：

- `use_sim_time:=true`：所有节点跟随 bag 的 `/clock`。
- `map_pcd:=...`：加载你刚建好的地图。
- `start_adapter:=false`：不向真机发送 `/NAV_CMD`。
- `rl_python_executable:=...`：使用已安装 GPU PyTorch 的 Python 3.12 环境。
- `pct_tomogram_file:=m20_3d_map`：加载 PCT 层析地图。

如果你的地图最终文件叫 `m20_map_leveled.pcd`，就把 `map_pcd` 攏成：

```bash
map_pcd:=$PWD/maps/fastlio/m20_map_leveled.pcd
```

## 2. RViz 中设置初始位姿

先不要急着播放 bag。

在 RViz 中：

```text
Fixed Frame = map
```

然后使用项目提供的 3D 初始位姿工具，或者 RViz 的 `2D Pose Estimate`，把机器狗初始位置放到地图中对应的位置。

这是在告诉定位系统：

```text
bag 开始时的机器狗，大约位于地图的这个位置。
```

如果初始位姿没有设置正确，常见表现是：

- 雷达点云与地图错开；
- 机器人突然跳到错误位置；
- `/pct_path` 有，但局部路径位置不对；
- `map -> odom` 变换不存在。

如果 bag 正是从建图起点开始录制，初始位置通常就在建图起点附近。

如果你使用的是旋平后的 `maps/fastlio/m20_3d_map.pcd`，这里尤其重要：不要让机器人继续沿原始倾斜地图的坐标系跑。正确现象是 `map -> odom` 会被 Open3D 全局定位拉到旋平地图坐标中，`base_link` 贴着旋平后的地图移动。如果 `base_link` 会动，但斜着跑出地图，先不要发目标，优先检查 Open3D 和初始位姿：

```bash
ros2 node list | grep global_localization
ros2 topic echo /localization_3d_confidence --once
ros2 topic echo /baselink2map --once
ros2 topic echo /odom2map --once
ros2 run tf2_ros tf2_echo map odom
```

判断方法：

- 没有 `/global_localization_node`：Open3D 全局定位没有启动。
- 没有 `/baselink2map` 或 `/odom2map`：Open3D 没有完成定位初始化，通常是没收到 `/Odometry_loc` 或 `/cloud_registered_1`。
- `map -> odom` 不变化：Open3D 没有把 Fast-LIO 的 odom 坐标拉回地图坐标。
- 置信度一直很低：初始位姿离真实起点太远，先暂停 bag，重新拖 `3D Initial Pose` 到起点附近。

## 3. 终端二：播放 bag

打开新终端：

```bash
cd ~/xlab/M20-loc-nav

export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=/tmp/m20_ros_logs

source /opt/ros/jazzy/setup.bash
source ~/xlab/liv_ws/install/setup.bash
source install/setup.bash
```

查看三个 bag：

```bash
ros2 bag list
ls bag
```

播放你用于建这张地图的同一个 bag，例如：

```bash
ros2 bag play bags/stair3 \
  --clock \
  --rate 0.5 \
  --start-paused
```

建议先用 `0.5` 倍速。这里带了 `--start-paused`，所以 bag 启动后默认是暂停状态，RViz 里的 `base_link` 不会动；需要在播放 bag 的终端按一次空格，或者通过服务继续播放：

```bash
ros2 service call /rosbag2_player/resume \
  rosbag2_interfaces/srv/Resume "{}"
```

之后也可以通过服务控制暂停和继续：

```bash
ros2 service call /rosbag2_player/toggle_paused \
  rosbag2_interfaces/srv/TogglePaused "{}"
```

查看播放器提供的服务：

```bash
ros2 service list | grep rosbag
```

如果没有 `toggle_paused`，分别使用：

```bash
ros2 service call /rosbag2_player/pause \
  rosbag2_interfaces/srv/Pause "{}"
```

```bash
ros2 service call /rosbag2_player/resume \
  rosbag2_interfaces/srv/Resume "{}"
```

`resume` 正常不会重播。如果位姿看起来回到原点，通常不是 bag 真正重播，而是定位节点重新初始化、`map -> odom` 丢失，或某个节点没有使用模拟时间。

如果播放时看到：

```text
New subscription discovered on topic '/clock', requesting incompatible QoS.
Last incompatible policy: RELIABILITY_QOS_POLICY
```

说明有节点用 reliable QoS 订阅 `/clock`，但 rosbag 的 `/clock` 发布端是 best-effort，订阅端收不到模拟时间。当前工程已把 `fastlio_odom_bridge` 的 `/clock` 订阅改成 best-effort；修改后需要重新编译并重新 source：

```bash
colcon build --symlink-install --packages-select m20_fastlio_nav
source install/setup.bash
```

确认 bag 已经真的在走：

```bash
ros2 topic hz /clock
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

`bags/stair3` 只包含 `/livox/lidar`、`/livox/imu` 和相机图像，不包含录好的 `/tf` 或 odom；因此 RViz 里的 `base_link` 运动必须由主链路实时跑 Fast-LIO 定位后生成。如果 `/clock`、`/livox/lidar`、`/livox/imu` 都有数据但 `base_link` 仍不动，继续查 `/Odometry_loc` 和 `/odom_body`。

## 4. 在 RViz 中添加显示项

点击 RViz 左下角：

```text
Add -> By topic
```

建议添加：

| Topic | RViz 显示类型 | 作用 |
|---|---|---|
| `/pct_path` | Path | PCT 生成的全局路径 |
| `/local_path` | Path | RL/PRIEST 实时局部路径 |
| `/local_path_rl_debug` | Path | 局部规划调试路径 |
| `/subgoal` | PoseStamped | Pure Pursuit 选出的局部目标 |
| `/scan` | LaserScan | 局部规划器看到的障碍物 |
| `/odom_body` | Odometry | 地图定位后的机器人状态 |
| `/Odometry_loc` | Odometry | Fast-LIO 里程计 |
| `/cloud_registered_body_1` | PointCloud2 | 当前雷达点云 |
| `/tf` | TF | 坐标系关系 |

颜色建议：

```text
/pct_path       绿色
/local_path     红色
/local_path_rl_debug  黄色
/scan           白色或蓝色
```

`/local_path` 的 `Line Style` 可以设为 `Billboards`，并适当增大 `Line Width`，这样更容易看清。

## 5. 给 PCT 一个地图目标

播放 bag 后，等待：

- 当前点云与地图基本重合；
- 机器狗位姿稳定；
- `map -> odom` 或对应定位 TF 已经建立。

然后在 RViz 中使用项目配置的目标点工具点击地图。

也可以直接从终端发布，例如：

```bash
export ROS_DOMAIN_ID=77
source /opt/ros/jazzy/setup.bash
source ~/xlab/liv_ws/install/setup.bash
source ~/xlab/M20-loc-nav/install/setup.bash

ros2 topic pub --once \
  /pct_goal_point \
  geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 5.0, y: 0.0, z: 0.5}}"
```

这里的坐标必须位于地图的可通行区域。不能机械地照抄 `x: 5.0`，需要根据 RViz 中地图的实际坐标选择。

目标发布后，数据链应该是：

```text
/pct_goal_point
      ↓
PCT
      ↓
/pct_path
      ↓
Pure Pursuit
      ↓
/subgoal
      ↓
RL + PRIEST
      ↓
/local_path
      ↓
DWB
      ↓
/cmd_vel
```

## 6. 终端三：检查每一级是否正常

打开第三个终端，加载相同环境：

```bash
cd ~/xlab/M20-loc-nav

export ROS_DOMAIN_ID=77
source /opt/ros/jazzy/setup.bash
source ~/xlab/liv_ws/install/setup.bash
source install/setup.bash
```

查看关键 Topic：

```bash
ros2 topic list | grep -E \
"global_localization|localization_3d|baselink2map|odom2map|pct_path|subgoal|local_path|scan|odom_body|Odometry_loc|cmd_vel"
```

依次验证：

```bash
ros2 node list | grep global_localization
ros2 topic echo /localization_3d_confidence --once
ros2 topic echo /baselink2map --once
ros2 topic echo /odom2map --once
ros2 topic echo /odom_body --once
ros2 topic echo /scan --once
ros2 topic echo /pct_path --once
ros2 topic echo /subgoal --once
ros2 topic echo /local_path --once
ros2 topic hz /local_path
```

正确顺序应该是：

1. `/odom_body` 有数据；
2. `/scan` 有数据；
3. 发布目标后 `/pct_path` 有数据；
4. `/subgoal` 有数据；
5. `/local_path` 持续更新。

## 如果没有 `/local_path`

按下面顺序查，不要一上来就怀疑 RViz：

```bash
ros2 topic echo /odom_body --once
ros2 topic echo /scan --once
ros2 topic echo /pct_path --once
ros2 topic echo /subgoal --once
```

判断方法：

- `/odom_body` 没有：地图定位没有运行成功。
- `/scan` 没有：点云转 LaserScan 链路有问题。
- `/pct_path` 没有：没有发布目标、tomogram 没生成，或 PCT 没启动。
- `/subgoal` 没有：Pure Pursuit 没启动，或 `/pct_path` 坐标系不对。
- 前面都有、唯独 `/local_path` 没有：检查 RL 节点、模型文件和 CUDA 环境。
- `/local_path` 有但 RViz 看不到：检查 RViz `Fixed Frame=map` 以及路径消息的 `header.frame_id`。

## 最少需要几个终端

建议三个：

```text
终端一：完整导航节点和 RViz
终端二：ros2 bag play
终端三：发目标、检查 Topic、暂停/继续
```

最重要的安全设置是：所有终端都使用 `ROS_DOMAIN_ID=77`，并保持 `start_adapter:=false`。这样可以实时观察 `/local_path` 和 `/cmd_vel`，但不会把导航指令发送给机器狗。
