import rclpy
import gymnasium as gym
import numpy as np

from rclpy.node import Node
from gymnasium import spaces

from .ros_gz_interface import *

class simplified_env(gym.Env, Node):
    """Custom Environment that follows gym interface."""

    def __init__(self, tolerence):
        super().__init__('simplified_env')
        self.get_logger().info("The simplified_env node has just been created.")
        self.ros_gz_interface = ros_gz_interface(self)
        self._tolerence = tolerence
        self._max_translational_velocity = np.float32(0.22)
        self._max_rotational_vel = np.float32(2.84)
        self._min_detection_distance = np.float32(0.16)
        self._max_detection_distance = np.float32(8.0) # Not used

        # Action space: 
        self.action_space = spaces.Box(
            low=np.array([0, -self._max_rotational_vel]),
            high=np.array([self._max_translational_velocity, self._max_rotational_vel]),
            dtype=np.float32
        )
        # Observation space: 
        obs_low = np.append(
            np.array([self._min_detection_distance]*360),
            np.array([np.float32(0), np.float32(-np.pi)])
        )
        obs_high = np.append(
            np.array([np.float32(9)]*360),
            np.array([np.float32(8.4853), np.float32(np.pi)])
        ) # (6**2 + 6**2)**0.5 = 8.4853
        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            dtype=np.float32
        )

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
        self.done = False
        self.truncated = False

        self._episode_count = 0
        self._step_count = 0

        self.laser_observation = np.array([np.float32(9)]*360)
        self.location_observation = np.array([0.0, 0.0])
        self.observation = np.append(self.laser_observation, self.location_observation)

    def step(self, action):
        # Increment step count
        self._step_count += 1

        # Publish an aciton (twist)
        self.ros_gz_interface.publish_twist(action)

        # Spin once
        self.spin()

        # Get observation and info
        observation = self.get_observation()
        info = self.get_info()

        # Get the reward
        reward = self.compute_rewards(info)

        # Check if the turtlebot reaches the goal with a tolerence
        self.done = (info["distance"] < self._tolerence)
        terminated = self.done

        # truncated is also False
        truncated = self.truncated

        return observation, reward, terminated, truncated, info

    async def reset(self, seed=None, options=None):
        # Reset RL variables
        self.done = False
        self.truncated = False

        self._step_count = 0

        self.laser_observation = np.array([np.float32(9)]*360)
        self.location_observation = np.array([0.0, 0.0])
        self.observation = np.append(self.laser_observation, self.location_observation)

        # Pause the simulation
        pause_request = Empty.Request()
        result = await self.control_simulation_cli(pause_request)

        # Re-initialize ros_gz_interface
        self.ros_gz_interface = ros_gz_interface(self)

        # Delete all obstacles in the scene
        delete_request = Empty.Request()
        result = await self.delete_all_obstacles_cli(delete_request)

        # Spawn all obstacles and move the turtlebot to its start pose
        self.ros_gz_interface.set_episode_num(self._episode_count)
        set_request = Empty.Request()
        result = await self.set_up_new_episode_cli(set_request)

        # Resume the simulation
        resume_request = Empty.Request()
        result = await self.control_simulation_cli(resume_request)

        # Spin once
        self.spin()

        # Get observation and info
        observation = self.get_observation()
        info = self.get_info()

        # Increment critical value
        self._episode_count += 1

        return observation, info

    def render(self):
        pass

    def close(self):
        self.destroy_client(self.control_simulation_cli)
        self.destroy_client(self.set_up_new_episode_cli)
        self.destroy_client(self.delete_all_obstacles_cli)
        self.ros_gz_interface.interface_destroy()
        self.destroy_node()

    def spin(self):
        rclpy.spin_once(self)

    def get_observation(self):
        self.laser_observation = self.ros_gz_interface.get_laser_readings()
        self.location_observation = self.ros_gz_interface.get_distance_and_bearing()
        self.observation = np.append(self.laser_observation, self.location_observation)
        return self.observation

    def get_info(self):
        return {
            "distance": self.location_observation[0],
            "bearing": self.location_observation[1]
        }

    def compute_rewards(self, info):
        reward = 0

        if info["distance"] < self._tolerence:
            reward += 2
        elif info["distance"] < 2 * self._tolerence:
            reward += 1

        if self.ros_gz_interface.obstacle_hit_penalty:
            reward += -1
        if self.ros_gz_interface.out_of_bound_penalty:
            reward += -3

        return reward


def main(args=None):
    rclpy.init(args=args)
    simplified_env_node = simplified_env()
    rclpy.spin(simplified_env_node)
    simplified_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
