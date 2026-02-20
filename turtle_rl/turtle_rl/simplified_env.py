import gymnasium as gym
import numpy as np
from gymnasium import spaces

class simplified_env(gym.Env):
    """Custom Environment that follows gym interface."""

    def __init__(self):
        super().__init__()


        # Action space: a discrete space for this simplified environment
        # [GO_NORTH, GO_SOUTH, GO_EAST, GO_WEST] for a unit velocity
        self.action_space = spaces.Discrete(n=4)
        # State space: 
        self.observation_space = spaces

    def step(self, action):
        pass

    def reset(self, seed=None, options=None):
        pass

    def render(self):
        pass

    def close(self):
        pass
