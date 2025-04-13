import os
from pathlib import Path
from typing import Optional

import gymnasium as gym
from gymnasium.wrappers.transform_observation import (
    FlattenObservation,
)

import rclpy
from stable_baselines3 import SAC
import tb4_drl_navigation.environments  # noqa: F401


class Example:
    def __init__(
            self,
            model: SAC,
            env: gym.Env,
            file_path: Optional[str] = None,
    ):

        self.model = model
        self.env = env
        self.file_path = file_path

    def train(self):
        self.model.learn(total_timesteps=10000, log_interval=4)
        self.model.save(self.file_path)

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
            except Exception as e:
                print('Unexpected error: ', str(e))


def main(args=None):
    rclpy.init(args=None)

    env = None
    try:
        env = gym.make('Turtlebot4Env-v0', world_name='static_world')
        env = FlattenObservation(env=env)

        # Training
        model = SAC('MlpPolicy', env, verbose=1)
        current_dir = Path(__file__).resolve().parent
        file_path = os.path.join(current_dir.resolve(), 'models', 'sac_example')

        train_example = Example(
            model=model, env=env, file_path=file_path
        )
        train_example.train()

        # After training is done, comment the training part and uncomment the ff.
        # model = SAC.load(file_path)
        # test_example = Example(
        #     model=model, env=env
        # )
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Unexpected error: {str(e)}')
    finally:
        if env is not None:
            env.close()


if __name__ == '__main__':
    main()
