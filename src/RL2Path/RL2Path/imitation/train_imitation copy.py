#!/usr/bin/env python3
"""
memory_efficient_bc.py
----------------------
行为克隆（Behaviour Cloning）训练脚本，针对大规模数据集做了内存优化：

1. hdf5 → torch Dataset：按需加载，不把全部数据一次性读入 RAM。
2. 抛弃 trajectory / flatten_trajectories，直接用 dataloader 供 BC 训练。
3. 保留自定义特征提取器接口，可替换为自家 CNN / MLP。
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict

import h5py
import numpy as np
import tomli
import torch
from torch.utils.data import DataLoader, Dataset

import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from imitation.algorithms.bc import BC
from imitation.util import logger as imit_logger

from RL2Path.model.custom_cnn import get_model  # 请根据实际路径调整


# -------------------------------------------------------------------------- #
# 数据集：懒加载 HDF5
# -------------------------------------------------------------------------- #
class H5TransitionDataset(Dataset):
    """零拷贝按需加载 (obs, act) 对。"""

    def __init__(self, h5_path: str | Path, action_dim: int, dtype: np.dtype = np.float32):
        self.h5_path = Path(h5_path)
        self.action_dim = action_dim
        self.dtype = dtype

        # 主进程先打开一次，worker 里会再打开（见 __getitem__）
        self._open_file()

        orig_obs_dim = self.obs_ds.shape[1]  # 原始观测维度
        self.obs_dim = 7500 + (orig_obs_dim - 2500)  # tile 前 2500 ×3，再拼剩余

    # ---- HDF5 句柄在各进程中独立维护 ---- #
    def _open_file(self):
        self.h5file = h5py.File(self.h5_path, "r", swmr=True)
        self.obs_ds = self.h5file["observations"]
        self.act_ds = self.h5file["actions"]

    # ---- Dataset 接口 ---- #
    def __len__(self) -> int:
        return len(self.obs_ds)

    def __getitem__(self, idx: int) -> Dict[str, np.ndarray]:
        # DataLoader 多进程时，子进程第一次调用需要重新打开文件
        if not hasattr(self, "h5file"):
            self._open_file()

        # np.astype(..., copy=False) 避免不必要复制
        obs = self.obs_ds[idx].astype(self.dtype, copy=False)
        act = self.act_ds[idx].astype(self.dtype, copy=False).reshape(-1)

        # 与旧脚本保持一致：前 2500 特征重复 3 次
        obs_processed = np.concatenate([np.tile(obs[:2500], 3), obs[2500:]], dtype=self.dtype)

        return {"obs": obs_processed, "acts": act}


# -------------------------------------------------------------------------- #
# 迷你环境（占位用）
# -------------------------------------------------------------------------- #
class PathFlowEnv(gym.Env):
    """仅用于占位，满足 SB3 接口。"""

    def __init__(self, obs_dim: int, action_dim: int):
        super().__init__()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=0.0, high=1.0, shape=(action_dim,), dtype=np.float32
        )

    def reset(self, *, seed: int | None = None, options: Dict[str, Any] | None = None):
        super().reset(seed=seed)
        return np.zeros(self.observation_space.shape, dtype=np.float32), {}

    def step(self, action):
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        reward, terminated, truncated = 0.0, True, False
        return obs, reward, terminated, truncated, {}


# -------------------------------------------------------------------------- #
# 主入口
# -------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Low‑memory Behaviour Cloning trainer")
    parser.add_argument(
        "--config",
        type=str,
        default="source/RL2Path/config/imitation.toml",
        help="Path to the TOML configuration file.",
    )
    return parser.parse_args()


def load_config(path: str | Path) -> Dict[str, Any]:
    with open(path, "rb") as f:
        return tomli.load(f)


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)

    imitation_cfg = cfg.get("imitation", {})
    paths_cfg = cfg["paths"]
    proc_cfg = cfg["process"]

    # -------------- 读取配置 -------------- #
    action_dim: int = proc_cfg["action_dim"]
    transition_h5_path: Path = Path(paths_cfg["output"])

    checkpoint_dir: Path = Path(imitation_cfg.get("checkpoint_dir", "./outputs/checkpoints"))
    log_dir: Path = Path(imitation_cfg.get("log_dir", "./outputs/logs"))
    resume: bool = imitation_cfg.get("resume", False)
    resume_model_path: str | None = imitation_cfg.get("resume_model_path")

    seed: int = imitation_cfg.get("seed", 42)
    batch_size: int = imitation_cfg.get("batch_size", 512)
    device: str = imitation_cfg.get("device", "cuda" if torch.cuda.is_available() else "cpu")
    model_name: str = imitation_cfg.get("model", "custom_cnn")
    feature_dim: int = imitation_cfg.get("feature_dim", 256)

    # -------------- 随机种子 -------------- #
    rng = np.random.default_rng(seed)
    torch.manual_seed(seed)

    # -------------- 数据集 & DataLoader -------------- #
    dataset = H5TransitionDataset(transition_h5_path, action_dim=action_dim, dtype=np.float32)
    dataloader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=16,          # 若确认 h5py 多进程 OK，可改 >0
        pin_memory=True,
    )

    # -------------- 伪环境 & Policy -------------- #
    env = PathFlowEnv(dataset.obs_dim, action_dim)

    policy_kwargs = dict(
        features_extractor_class=get_model(model_name),
        features_extractor_kwargs=dict(features_dim=feature_dim),
    )

    if resume and resume_model_path:
        model = PPO.load(resume_model_path, env=env, device=device)
        start_epoch = int(Path(resume_model_path).stem.split("_")[-1])
    else:
        model = PPO(
            "MlpPolicy",
            env,
            policy_kwargs=policy_kwargs,
            verbose=1,
            device=device,
            batch_size=2048,
        )
        start_epoch = 0

    # -------------- Logger -------------- #
    bc_logger = imit_logger.configure(folder=str(log_dir), format_strs=["tensorboard"])

    # -------------- BC 训练器 -------------- #
    bc_trainer = BC(
        rng=rng,
        observation_space=env.observation_space,
        action_space=env.action_space,
        policy=model.policy,
        demonstrations=dataloader,
        batch_size=batch_size,
        device=device,
        custom_logger=bc_logger,
    )

    # -------------- 训练循环 -------------- #
    total_epochs = 20
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    try:
        for epoch in range(start_epoch, start_epoch + total_epochs):
            bc_trainer.train(n_epochs=1)
            ckpt_path = checkpoint_dir / f"model_{epoch + 1}"
            model.save(str(ckpt_path))
            print(f"[INFO] checkpoint saved → {ckpt_path}")

        print("✨ training finished ✨")
        
    except KeyboardInterrupt:
        print("\nTraining interrupted by user. Saving current model...")
        ckpt_path = checkpoint_dir / f"model_{epoch + 1}"
        model.save(str(ckpt_path))
        print(f"[INFO] checkpoint saved → {ckpt_path}")


if __name__ == "__main__":
    main()
