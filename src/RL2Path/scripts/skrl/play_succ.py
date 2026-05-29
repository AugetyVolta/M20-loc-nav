# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to play a checkpoint of an RL agent from skrl.

Visit the skrl documentation (https://skrl.readthedocs.io) to see the examples structured in
a more user-friendly way.
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Play a checkpoint of an RL agent from skrl.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to model checkpoint.")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument(
    "--ml_framework",
    type=str,
    default="torch",
    choices=["torch", "jax", "jax-numpy"],
    help="The ML framework used for training the skrl agent.",
)
parser.add_argument(
    "--algorithm",
    type=str,
    default="PPO",
    choices=["AMP", "PPO", "IPPO", "MAPPO"],
    help="The RL algorithm used for training the skrl agent.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
parser.add_argument("--num_episodes", type=int, default=100, help="Number of episodes to evaluate for success rate.")
parser.add_argument("--sb3_checkpoint", type=str, default=None, help="Path to SB3 model.zip for warm-starting skrl")

parser.add_argument("--imitation_cfg", type=str, default="source/RL2Path/config/imitation.toml",
                    help="Path to imitation config file (for RL2Path imitation training).")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import time
import torch
import pickle
import re
import yaml
import tomli

import skrl
from packaging import version
from collections import OrderedDict

# check for minimum supported skrl version
SKRL_VERSION = "1.4.2"
if version.parse(skrl.__version__) < version.parse(SKRL_VERSION):
    skrl.logger.error(
        f"Unsupported skrl version: {skrl.__version__}. "
        f"Install supported version using 'pip install skrl>={SKRL_VERSION}'"
    )
    exit()

if args_cli.ml_framework.startswith("torch"):
    from skrl.utils.runner.torch import Runner
elif args_cli.ml_framework.startswith("jax"):
    from skrl.utils.runner.jax import Runner

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.utils.dict import print_dict
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.skrl import SkrlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path, load_cfg_from_registry, parse_env_cfg

import RL2Path.tasks  # noqa: F401

from RL2Path.model.policy import PolicyNet
from RL2Path.model.value import ValueNet
from skrl.agents.torch.ppo import PPO
from stable_baselines3 import PPO as SB3PPO



# config shortcuts
algorithm = args_cli.algorithm.lower()

def get_name_map(skrl_model, sb3_model):
    sd_dst = skrl_model.state_dict()
    sd_src = sb3_model.policy.state_dict()
    
    name_map = OrderedDict()

    # ---------- 1. 先做“完全相同名字”的配对 ----------
    for k in sd_dst.keys():
        if k in sd_src:
            name_map[k] = k

    # ---------- 2. 用前缀规则补齐 ----------
    prefix_rules = [
        # a) SB3 的 log_std → skrl 的 log_std_parameter
        (r"^log_std$",                     "log_std_parameter"),
        # b) mlp_extractor.policy_net.* → policy_net.*
        (r"^mlp_extractor\.policy_net\.",  "policy_net."),
        # c) mlp_extractor.value_net.*  → value_net.*
        (r"^mlp_extractor\.value_net\.",   "value_net."),
        # d) SB3 最后的 value_net.*     → skrl 的 value_output.*
        (r"^value_net\.",                  "value_output.")
    ]

    for src_key in sd_src.keys():
        for pat_src, repl_dst in prefix_rules:
            if re.match(pat_src, src_key):
                dst_key = re.sub(pat_src, repl_dst, src_key)
                # 避免重复覆盖，且确保目标模型里确实有这个键
                if dst_key in sd_dst and dst_key not in name_map:
                    name_map[dst_key] = src_key

    return name_map

def main():
    """Play with skrl agent."""
    # configure the ML framework into the global skrl variable
    if args_cli.ml_framework.startswith("jax"):
        skrl.config.jax.backend = "jax" if args_cli.ml_framework == "jax" else "numpy"

    task_name = args_cli.task.split(":")[-1]

    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    try:
        experiment_cfg = load_cfg_from_registry(task_name, f"skrl_{algorithm}_cfg_entry_point")
    except ValueError:
        experiment_cfg = load_cfg_from_registry(task_name, "skrl_cfg_entry_point")

    # specify directory for logging experiments (load checkpoint)
    log_root_path = os.path.join("logs", "skrl", experiment_cfg["agent"]["experiment"]["directory"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    # get checkpoint path
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("skrl", task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = os.path.abspath(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(
            log_root_path, run_dir=f".*_{algorithm}_{args_cli.ml_framework}", other_dirs=["checkpoints"]
        )
    log_dir = os.path.dirname(os.path.dirname(resume_path))

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv) and algorithm in ["ppo"]:
        env = multi_agent_to_single_agent(env)

    # get environment (step) dt for real-time evaluation
    try:
        dt = env.step_dt
    except AttributeError:
        dt = env.unwrapped.step_dt

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for skrl
    env = SkrlVecEnvWrapper(env, ml_framework=args_cli.ml_framework)  # same as: `wrap_env(env, wrapper="auto")`

    # configure and instantiate the skrl runner
    # https://skrl.readthedocs.io/en/latest/api/utils/runner.html
    experiment_cfg["trainer"]["close_environment_at_exit"] = False
    experiment_cfg["agent"]["experiment"]["write_interval"] = 0  # don't log to TensorBoard
    experiment_cfg["agent"]["experiment"]["checkpoint_interval"] = 0  # don't generate checkpoints
    # runner = Runner(env, experiment_cfg)
    
    agent_path = os.path.join(log_dir, "params", "agent.pkl")
    with open(agent_path, "rb") as f:
        agent_cfg = pickle.load(f)
    
    with open(args_cli.imitation_cfg, 'rb') as f:
        imitation_cfg = tomli.load(f)['imitation']
    goal_include_theta = imitation_cfg.get('goal_include_theta', True)
        
    print(f"[INFO] policy goal include theta: {goal_include_theta}")
        
    policy_net = PolicyNet(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.sim.device,
        clip_actions=False,
        goal_include_theta=goal_include_theta
    )    
    
    print(f"[INFO] value goal include theta: {goal_include_theta}")
    
    value_net = ValueNet(
        observation_space=env.observation_space,
        action_space=1,
        device=env.sim.device,
        clip_actions=False,
        goal_include_theta=goal_include_theta
    )    
    
    if args_cli.sb3_checkpoint is not None:
        sb3_model = SB3PPO.load(args_cli.sb3_checkpoint)
        
        policy_name_map = get_name_map(policy_net, sb3_model)
        value_name_map = get_name_map(value_net, sb3_model)
        
        policy_net.migrate(path=args_cli.sb3_checkpoint, name_map=policy_name_map, verbose=False)
        value_net.migrate(path=args_cli.sb3_checkpoint, name_map=value_name_map, verbose=False)
        
    models = {"policy": policy_net, "value": value_net}
    
    agent = PPO(
        models=models,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.sim.device,
        cfg=agent_cfg,
    )
    
    if not args_cli.sb3_checkpoint:
        agent.load(resume_path)
        
    agent.policy.eval()
    
    # runner.agent.load(resume_path)
    # set agent to evaluation mode
    # runner.agent.set_running_mode("eval")
    
    target_episodes = args_cli.num_episodes
    reward_threshold = 0.4
    success_count = 0
    total_episodes = 0
    timestep = 0

    obs, _ = env.reset()

    while total_episodes < target_episodes and simulation_app.is_running():
        start_time = time.time()

        with torch.inference_mode():
            outputs = agent.act(obs, timestep=0, timesteps=0)

            if hasattr(env, "possible_agents"):
                actions = {a: outputs[-1][a].get("mean_actions", outputs[0][a]) for a in env.possible_agents}
            else:
                actions = outputs[-1].get("mean_actions", outputs[0])

            obs, reward, terminated, truncated, info = env.step(actions)

            # make reward, terminated, info lists if not already
            rewards = reward.view(-1).tolist()
            successes = (reward >= reward_threshold).view(-1).tolist()

            for i, (r, s) in enumerate(zip(rewards, successes)):
                success_count += int(s)
                total_episodes += 1

                print(
                    f"[Episode {total_episodes:03d}, Env {i}] "
                    f"Reward: {r:.4f} | Success: {bool(s)} | "
                    f"Success Count: {success_count}"
                )

                if total_episodes >= target_episodes:
                    break

        # reset for next episode
        obs, _ = env.reset()

        if args_cli.video:
            timestep += 1
            if timestep >= args_cli.video_length:
                break

        # real-time delay
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    env.close()

    success_rate = success_count / target_episodes
    print(f"\n[INFO] Episodes: {target_episodes} | Successes: {success_count} | Success Rate: {success_rate:.2%}")


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
