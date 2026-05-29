# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from Stable-Baselines3."""

"""Launch Isaac Sim Simulator first."""

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Play a checkpoint of an RL agent from Stable-Baselines3.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during evaluation.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to model checkpoint.")
parser.add_argument("--use_pretrained_checkpoint", action="store_true", help="Use pre-trained checkpoint from Nucleus.")
parser.add_argument("--use_last_checkpoint", action="store_true", help="Use last saved model instead of best.")
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
parser.add_argument("--keep_all_info", action="store_true", default=False, help="Keep extra training info (slower).")
parser.add_argument("--num_episodes", type=int, default=100, help="Number of episodes to evaluate for success rate.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse args
args_cli = parser.parse_args()
# always enable cameras if recording
if args_cli.video:
    args_cli.enable_cameras = True

# launch Omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import time
import torch

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.utils.dict import print_dict
from isaaclab.utils.io import load_yaml
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.sb3 import Sb3VecEnvWrapper, process_sb3_cfg

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import get_checkpoint_path, parse_env_cfg

import RL2Path.tasks  # noqa: F401


def main():
    """Evaluate SB3 agent and calculate success rate."""
    # load env config
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    task_name = args_cli.task.split(":")[-1]
    log_root_path = os.path.abspath(os.path.join("logs", "sb3", task_name))

    # resolve checkpoint path
    if args_cli.use_pretrained_checkpoint:
        checkpoint_path = get_published_pretrained_checkpoint("sb3", task_name)
        if not checkpoint_path:
            print("[INFO] Pre-trained checkpoint unavailable.")
            return
    elif args_cli.checkpoint is None:
        checkpoint = "model_.*.zip" if args_cli.use_last_checkpoint else "model.zip"
        checkpoint_path = get_checkpoint_path(log_root_path, ".*", checkpoint)
    else:
        checkpoint_path = args_cli.checkpoint
    log_dir = os.path.dirname(checkpoint_path)

    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # record video if needed
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap for SB3
    env = Sb3VecEnvWrapper(env, fast_variant=not args_cli.keep_all_info)

    vec_norm_path = Path(checkpoint_path.replace("/model", "/model_vecnormalize").replace(".zip", ".pkl"))

    # (Optional) normalization
    # if vec_norm_path.exists():
    #     print(f"Loading saved normalization: {vec_norm_path}")
    #     env = VecNormalize.load(vec_norm_path, env)
    #     env.training = False
    #     env.norm_reward = False

    # load agent
    print(f"Loading checkpoint from: {checkpoint_path}")
    agent = PPO.load(checkpoint_path, env, print_system_info=True)

    # success rate evaluation
    target_episodes = args_cli.num_episodes  # e.g. 200
    success_count = 0
    total_episodes = 0
    reward_threshold = 0.4

    while total_episodes < target_episodes:
        obs = env.reset()
        with torch.inference_mode():
            action, _ = agent.predict(obs, deterministic=True)
        obs, _, terminated, info = env.step(action)

        if isinstance(info, list):
            for env_idx, i in enumerate(info):
                reward = i.get("episode", {}).get("r", 0.0)
                success = reward >= reward_threshold
                success_count += int(success)
                total_episodes += 1

                print(
                    f"[Episode {total_episodes:03d}, Env {env_idx}] "
                    f"Reward: {reward:.4f} | Success: {success} | "
                    f"Success Count: {success_count}"
                )

                if total_episodes >= target_episodes:
                    break

    env.close()

    # report success rate
    success_rate = success_count / target_episodes
    print(f"\n[INFO] Episodes: {target_episodes} | Successes: {success_count} | Success Rate: {success_rate:.2%}")


if __name__ == "__main__":
    main()
    simulation_app.close()
