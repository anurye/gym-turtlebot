import argparse
from dataclasses import asdict, dataclass, field
from pathlib import Path
import random
import sys
from typing import Any, Dict, List, Optional, Tuple

import gymnasium as gym
import numpy as np

import rclpy
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CheckpointCallback,
    EvalCallback
)
from stable_baselines3.common.monitor import Monitor
import tb4_drl_navigation.envs  # noqa: F401
import torch
import torch.nn as nn
import yaml


@dataclass(frozen=True)
class EnvConfig:
    env_id: str = 'Turtlebot4Env-v0'
    world_name: str = 'static_world'
    robot_name: str = 'turtlebot4'
    obstacle_prefix: str = 'obstacle'
    robot_radius: float = 0.3
    obstacle_clearance: float = 2.0
    min_separation: float = 1.5
    goal_sampling_bias: str = 'close'
    num_bins: int = 20
    goal_threshold: float = 0.35
    collision_threshold: float = 0.4
    time_delta: float = 0.4
    shuffle_on_reset: bool = True

    map_path: Optional[Path] = None
    yaml_path: Optional[Path] = None
    sim_launch_path: Optional[Path] = None


@dataclass(frozen=True)
class SACConfig:
    # Algorithm
    policy_type: str = 'MlpPolicy'
    buffer_size: int = 1_000_000
    batch_size: int = 256
    gamma: float = 0.99
    tau: float = 0.005
    target_update_interval: int = 1
    learning_rate: float = 3e-4
    train_freq: Tuple[int, str] = (1, 'step')
    gradient_steps: int = 1

    # Network
    policy_kwargs: Dict[str, Any] = field(
        default_factory=lambda: {
            'net_arch': {'pi': [512, 512, 256], 'qf': [1024, 1024, 512]},
            'activation_fn': nn.ReLU,
            'optimizer_class': torch.optim.Adam,
        }
    )


@dataclass(frozen=True)
class ExperimentConfig:
    env: EnvConfig = field(default_factory=EnvConfig)
    sac: SACConfig = field(default_factory=SACConfig)

    total_timesteps: int = 1_000_000
    save_freq: int = 50_000
    eval_freq: int = 50_000
    n_eval_episodes: int = 5
    seed: Optional[int] = 42
    use_deterministic_cudnn: bool = False

    log_dir: Path = Path('experiments')
    experiment_name: str = 'sac_navigation'


def make_env(config: ExperimentConfig) -> gym.Env:
    rclpy.init(args=None)
    env_params = asdict(config.env)
    env = gym.make(
        env_params.pop('env_id'),
        **env_params
    )
    env = gym.wrappers.FlattenObservation(env)
    return Monitor(env, filename=str(config.log_dir / config.experiment_name / 'monitor.csv'))


class SACExperiment:
    def __init__(self, env: gym.Env, config: ExperimentConfig):
        self.config = config
        self.env = env

        self.experiment_path = self.config.log_dir / self.config.experiment_name
        self.checkpoints_path = self.experiment_path / 'checkpoints'
        self.best_model_path = self.experiment_path / 'best_model'
        self.logs_path = self.experiment_path / 'logs'
        self.eval_logs_path = self.experiment_path / 'eval_logs'
        self.final_model = self.experiment_path / 'final_model'

        self.experiment_path.mkdir(parents=True, exist_ok=True)
        self.checkpoints_path.mkdir(exist_ok=True)
        self.best_model_path.mkdir(exist_ok=True)
        self.logs_path.mkdir(exist_ok=True)
        self.eval_logs_path.mkdir(exist_ok=True)

        self._set_seeds()

        sac_params = asdict(self.config.sac)
        self.model = SAC(
            policy=sac_params.pop('policy_type'),
            env=self.env,
            **sac_params,
            verbose=1,
            tensorboard_log=str(self.experiment_path / 'logs'),
            device='auto',
        )

        print(self.model.policy)

    def _set_seeds(self):
        if self.config.seed is not None:
            random.seed(self.config.seed)
            np.random.seed(self.config.seed)
            torch.manual_seed(self.config.seed)
            if self.config.use_deterministic_cudnn:
                torch.backends.cudnn.deterministic = True

    def _get_callbacks(self) -> List[BaseCallback]:
        return [
            CheckpointCallback(
                save_freq=self.config.save_freq,
                save_path=str(self.checkpoints_path),
                name_prefix='sac_model'
            ),
            EvalCallback(
                eval_env=self.env,
                best_model_save_path=str(self.best_model_path),
                log_path=str(self.eval_logs_path),
                eval_freq=self.config.save_freq,
                n_eval_episodes=self.config.n_eval_episodes,
                deterministic=True
            )
        ]

    def train(self):
        try:
            self.model.learn(
                total_timesteps=self.config.total_timesteps,
                callback=self._get_callbacks(),
                log_interval=25,
                progress_bar=True,
                tb_log_name='sac_training'
            )
        except KeyboardInterrupt:
            pass
        finally:
            self.model.save(self.final_model)
            config_path = self.experiment_path / 'config.yaml'
            with open(config_path, 'w') as f:
                yaml.dump(asdict(self.config), f, sort_keys=False)

            self.env.close()


class SACInference:
    def __init__(self, env: gym.Env, model_path: Path):
        self.model = SAC.load(model_path)
        self.env = env

    def run(self, num_episodes: int = 10):
        try:
            for _ in range(num_episodes):
                obs, _ = self.env.reset()
                while True:
                    action, _ = self.model.predict(obs, deterministic=True)
                    obs, _, terminated, truncated, _ = self.env.step(action)

                    if terminated or truncated:
                        break
        except KeyboardInterrupt:
            pass
        finally:
            self.env.close()


def main():
    parser = argparse.ArgumentParser(
        description='Turtlebot4 Navigation with SAC Example',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    subparsers = parser.add_subparsers(dest='command')

    train_parser = subparsers.add_parser('train')
    train_parser.add_argument('--config', type=Path, help='Path to config file')

    eval_parser = subparsers.add_parser('eval')
    eval_parser.add_argument('model_path', type=Path)
    eval_parser.add_argument('--episodes', type=int, default=10)

    args = parser.parse_args()

    config = ExperimentConfig()

    env = make_env(config=config)
    if args.command == 'train':
        experiment = SACExperiment(env=env, config=config)
        experiment.train()
    elif args.command == 'eval':
        args.model_path = args.model_path or (
            Path(__file__).parent / config.log_dir / config.experiment_name / 'final_model'
        )
        inference = SACInference(env=env, model_path=args.model_path)
        inference.run(args.episodes)
    else:
        parser.print_help()
        sys.exit(-1)


if __name__ == '__main__':
    main()
