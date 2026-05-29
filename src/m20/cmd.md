sudo systemctl stop planner.service
sudo systemctl stop localization.service

# time assign
## remote
sudo systemctl restart chrony
## local
sudo systemctl restart chrony
sudo chronyc makestep

# Livox Mid360
cd ~/liv_ws
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# M20

ros2 launch m20 m20_launch.launch.py

ros2 launch nav2_bringup rviz_launch.py use_sim_time:=False

ros2 launch nav2_bringup slam_launch.py params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_slam.yaml use_sim_time:=False

ros2 launch nav2_bringup navigation_launch.py params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2.yaml use_sim_time:=False

ros2 launch nav2_bringup navigation_launch.py params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2_real.yaml use_sim_time:=False

python3 src/move/move/global_path_publisher.py

<!-- conda activate dog -->
source ~/venv/m20_nav/bin/activate

python /home/orin/workspace/m20_ws/src/move/move/priest_rl_publisher_nav_cmd.py
python /home/orin/workspace/m20_ws/src/move/move/priest_mppi_adapter_nav_cmd.py
python3 /home/orin/workspace/m20_ws/src/move/move/pure_pursuit.py

# localization and nav
ros2 launch nav2_bringup localization_launch.py use_sim_time:=False map:=/home/orin/workspace/m20_ws/src/move/map/lab.yaml params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2_real.yaml
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2_real.yaml use_sim_time:=False
# 和上面两条等价
ros2 launch nav2_bringup bringup_launch.py map:=/home/orin/workspace/m20_ws/src/move/map/lab.yaml params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2_real.yaml use_sim_time:=False

ros2 launch nav2_bringup bringup_launch.py map:=/home/orin/workspace/m20_ws/src/move/map/lab.yaml params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2_real_bt.yaml use_sim_time:=False

ros2 launch nav2_bringup bringup_launch.py map:=/mnt/nvme/workspace/fast_lio_ws/maps/fastlio/m20_2d_map.yaml params_file:=/mnt/nvme/workspace/m20_ws/src/m20/config/m20_nav2_real_dwb_smooth_responsive.yaml use_sim_time:=False rviz:=true


ros2 launch m20 m20_nav2_minimal_launch.launch.py \
  params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2_real.yaml \
  use_sim_time:=False

python3 src/move/move/global_path_publisher.py

source ~/venv/m20_nav/bin/activate
python /home/orin/workspace/m20_ws/src/move/move/priest_rl_publisher_nav_cmd.py
python /home/orin/workspace/m20_ws/src/move/move/priest_rl_publisher_nav_cmd_fast.py

python /home/orin/workspace/m20_ws/src/move/move/priest_mppi_adapter_nav_cmd.py
python /mnt/nvme/workspace/m20_ws/src/move/move/priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py

python3 /home/orin/workspace/m20_ws/src/move/move/pure_pursuit.py


# use classic controller
ros2 launch nav2_bringup bringup_launch.py map:=/home/orin/workspace/m20_ws/src/move/map/lab.yaml params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2_dwb.yaml use_sim_time:=False
ros2 launch nav2_bringup bringup_launch.py map:=/home/orin/workspace/m20_ws/src/move/map/lab.yaml params_file:=/home/orin/workspace/m20_ws/src/m20/config/m20_nav2_mppi.yaml use_sim_time:=False