# point cloud
./isaacsim/python.sh source/RL2Path/RL2Path/imitation/point_cloud_collect.py --config source/RL2Path/config/imitation.toml

./isaacsim/python.sh source/RL2Path/RL2Path/imitation/test_lidar.py --config source/RL2Path/config/imitation_test.toml

# demonstration
./isaacsim/python.sh  source/RL2Path/RL2Path/imitation/demonstration_collect.py --config source/RL2Path/config/imitation.toml

./isaacsim/python.sh  source/RL2Path/RL2Path/imitation/demonstration_collect_direct.py --config source/RL2Path/config/imitation.toml

./isaacsim/python.sh  source/RL2Path/RL2Path/imitation/test.py --mode 2d --lidar_dataset /mnt/nas_7/datasets/NavScene/lidar_dataset_0.4_2d.h5 --path_dataset /mnt/nas_9/group/zhouboyang/merged_data_2d_0.4.npz --output /mnt/nas_7/datasets/NavScene/demonstrations_test.h5 --action_dim 10 --sample_n 50

# imitation
./isaacsim/python.sh  source/RL2Path/RL2Path/imitation/train_imitation.py --config source/RL2Path/config/imitation.toml

# rl
./isaacsim/python.sh -m torch.distributed.run --nnodes=1 --nproc_per_node=2 scripts/skrl/resume_train.py --task=Template-Rl2path-Direct-v0 --headless --distributed --sb3_checkpoint outputs/ckpts/tcnn_v0/exp_13/model_7.zip --imitation_cfg source/RL2Path/config/imitation.toml --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" --num_envs=1536

./isaacsim/python.sh scripts/skrl/resume_train.py --task=Template-Rl2path-Direct-v0 --sb3_checkpoint outputs/ckpts/tcnn_v0/model_7.zip  --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" --num_envs=1

./isaacsim/python.sh scripts/skrl/train.py --task=Template-Rl2path-Direct-v0 --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" --num_envs=1

# play
./isaacsim/python.sh scripts/skrl/play.py --task Template-Rl2path-Direct-v0 --checkpoint logs/skrl/rl2path_direct/2025-07-15_20-52-04_ppo_torch/checkpoints/best_agent.pt --num_envs 1 --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error"

./isaacsim/python.sh scripts/sb3/play.py --task Template-Rl2path-Direct-v0 --checkpoint outputs/ckpts/tcnn_v0/model_7.zip --num_envs 1 --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error"

# test succ
./isaacsim/python.sh scripts/sb3/play_succ.py --task Template-Rl2path-Direct-v0 --checkpoint outputs/ckpts/tcnn_v0/exp_13/model_7.zip --num_envs 10 --num_episodes 400 --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" 

./isaacsim/python.sh scripts/skrl/play_succ.py --task Template-Rl2path-Direct-v0 --checkpoint logs/skrl/rl2path_direct/2025-07-21_23-44-05_ppo_torch/checkpoints/best_agent.pt --num_envs 20 --num_episodes 400 --imitation_cfg source/RL2Path/config/imitation.toml --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error"

./isaacsim/python.sh scripts/skrl/play_succ.py --task Template-Rl2path-Direct-v0 --sb3_checkpoint outputs/ckpts/tcnn_v0/model_7.zip --checkpoint logs/skrl/rl2path_direct/reset_exp_0/checkpoints/agent_10.pt --num_envs 10 --num_episodes 400 --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error"

# tensorboard
./isaacsim/python.sh -m tensorboard.main --logdir=logs/skrl/rl2path_direct/2025-07-16_16-46-05_ppo_torch

# -----------------------------------

./isaacsim/python.sh  source/RL2Path/RL2Path/imitation/demonstration_collect.py --mode 2d --lidar_dataset /mnt/nas_7/datasets/NavScene/lidar_dataset_2d.h5 --path_dataset /mnt/nas_9/group/zhouboyang/merged_data_2d.npz --output /mnt/nas_7/datasets/NavScene/demonstrations.h5 --action_dim 10 --headless --sample_n 10

./isaacsim/python.sh  source/RL2Path/RL2Path/imitation/demonstration_collect.py --mode 2d --lidar_dataset /mnt/nas_7/datasets/NavScene/lidar_dataset_2d.h5 --path_dataset /mnt/nas_9/group/zhouboyang/merged_data_2d.npz --output ./outputs/test.h5 --action_dim 10 --headless --sample_n 10

gpath: (array([ 8.917143, -6.970084], dtype=float32), array([ 9.294598, -7.421738], dtype=float32))

./isaacsim/python.sh scripts/sb3/resuming_training.py --task=Template-Rl2path-Direct-v0 --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" --pretrained_path outputs/ckpts/model_7.zip

./isaacsim/python.sh  source/RL2Path/RL2Path/imitation/test.py --mode 2d --lidar_dataset /mnt/nas_7/datasets/NavScene/lidar_dataset_0.5_2d.h5 --path_dataset /mnt/nas_9/group/zhouboyang/merged_data_2d_0.5.npz --output /mnt/nas_7/datasets/NavScene/demonstrations_test.h5 --action_dim 10 --headless --sample_n 50

./isaacsim/python.sh  -m torch.distributed.run --nnodes=1 --nproc_per_node=2 scripts/skrl/resume_train.py --task=Template-Rl2path-Direct-v0 --headless --sb3_checkpoint outputs/ckpts/tcnn_v0/model_20.zip --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" --distributed

./isaacsim/python.sh  source/RL2Path/RL2Path/imitation/demonstration_collect.py --config

./isaacsim/python.sh -m torch.distributed.run --nnodes=1 --nproc_per_node=2 scripts/skrl/resume_train.py --task=Template-Rl2path-Direct-v0 --headless --distributed --sb3_checkpoint outputs/ckpts/tcnn_v0/model_7.zip  --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" --num_envs=2048



