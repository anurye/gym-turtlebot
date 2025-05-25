from pathlib import Path
from typing import Optional

import gymnasium as gym
from gymnasium.wrappers.transform_observation import (
    FlattenObservation,
)

import rclpy
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor  # noqa: F401
import tb4_drl_navigation.envs  # noqa: F401

TRAIN = True


class Example:
    def __init__(
            self,
            model: SAC,
            env: gym.Env,
            file_path: Optional[Path] = None,
    ):
        self.model = model
        self.env = env
        self.file_path = file_path

    def train(self):
        try:
            self.model.learn(total_timesteps=1_0000_000, log_interval=4)
        except KeyboardInterrupt:
            pass
        finally:
            self.model.save(self.file_path)
            self.env.close()

    def test(self):
        obs, info = self.env.reset()
        while True:
            try:
                action, _states = self.model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = self.env.step(action=action)
                if terminated or truncated:
                    obs, info = self.env.reset()
            except KeyboardInterrupt:
                pass
            finally:
                self.env.close()


def main(args=None):
    rclpy.init(args=args)

    env = gym.make('Turtlebot4Env-v0', world_name='static_world')
    env = FlattenObservation(env=env)
    # env = Monitor(env=env, filename='monitor.csv', allow_early_resets=True)

    if TRAIN:
        model = SAC('MlpPolicy', env, verbose=1)
        current_dir = Path(__file__).resolve().parent
        file_path = current_dir.resolve() / 'models' / 'sac_example'

        train_example = Example(
            model=model, env=env, file_path=file_path
        )
        train_example.train()
    else:
        model = SAC.load(file_path)
        test_example = Example(model=model, env=env)
        test_example.test()

    env.close()


if __name__ == '__main__':
    main()
