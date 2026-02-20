import rclpy
import gymnasium as gym
import numpy as np

from rclpy.node import Node
from gymnasium import spaces

from .ros_gz_interface import *

class simplified_env(gym.Env, Node):
    """Custom Environment that follows gym interface."""

    def __init__(self):
        super().__init__('simplified_env')
        self.get_logger().info("The simplified_env node has just been created.")
        self.ros_gz_interface = ros_gz_interface(self)

        # Action space: 
        self.action_space = spaces.Box()
        # Observation space: 
        self.observation_space = spaces.Box()

        # Clients
        self._se_callback_group = MutuallyExclusiveCallbackGroup()

        self.control_simulation_cli = self.create_client(
            Empty,
            '/control_simulation',
            callback_group=self._se_callback_group
        )
        while not self.control_simulation_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for control simulation client to be available...")

        self.set_up_new_episode_cli = self.create_client(
            Empty,
            '/set_up_new_episode',
            callback_group=self._se_callback_group
        )
        while not self.set_up_new_episode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set up new episode client to be available...")

        self.delete_all_obstacles_cli = self.create_client(
            Empty,
            '/delete_all_obstacles',
            callback_group=self._se_callback_group
        )
        while not self.delete_all_obstacles_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for delete all obstacles client to be available...")

        # RL variables
        self._step_count = 0
        self._episode_count = 0

    def step(self, action):
        pass

    def reset(self, seed=None, options=None):
        pass
        # return observation, info

    def render(self):
        pass

    def close(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    simplified_env_node = simplified_env()
    rclpy.spin(simplified_env_node)
    simplified_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
