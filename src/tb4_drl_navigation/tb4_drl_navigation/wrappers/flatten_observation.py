import gymnasium as gym
from gymnasium import spaces
import numpy as np


class FlattenObservation(gym.ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
        assert isinstance(env.observation_space, spaces.Dict), 'Only Dict spaces supported'

        # Preserve the order of keys as defined in the original observation space
        self._key_order = list(env.observation_space.keys())

        # Calculate concatenated low/high
        lows = []
        highs = []
        for key in self._key_order:
            subspace = env.observation_space[key]
            assert isinstance(subspace, spaces.Box), 'Subspaces must be Box'
            lows.append(subspace.low.flatten())
            highs.append(subspace.high.flatten())

        low = np.concatenate(lows).astype(np.float32)
        high = np.concatenate(highs).astype(np.float32)

        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

    def observation(self, obs: spaces.Dict):
        # Concatenate in the preserved key order
        # print('Obs in wrapper:', obs)
        return np.concatenate([obs[key].flatten() for key in self._key_order])
