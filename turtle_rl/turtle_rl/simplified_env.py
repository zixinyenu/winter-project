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
        # TODO Make it a ROS parameter
        self._tolerence = 0.25
        self._max_translational_velocity = np.float32(0.22)
        self._max_rotational_vel = np.float32(2.84)
        self._min_detection_distance = np.float32(0.16)
        self._max_detection_distance = np.float32(8.0) # Not used

        # Action space: 
        # self.action_space = spaces.Box(
        #     low=np.array([np.float32(0), -self._max_rotational_vel]),
        #     high=np.array([self._max_translational_velocity, self._max_rotational_vel]),
        #     dtype=np.float32
        # )
        self.action_space = spaces.MultiDiscrete(
            nvec=np.array([12, 17]),
            dtype=np.uint8
        )
        # Observation space: 
        obs_low = np.append(
            np.array([self._min_detection_distance - np.float32(0.1)]*360),
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

        # RL variables
        self.done = False
        self.truncated = False

        self._step_count = 0

        self.laser_observation = np.array([np.float32(9)]*360)
        self.location_observation = np.array([0.0, 0.0])
        self.observation = np.append(self.laser_observation, self.location_observation)

    def step(self, action):
        # Increment step count
        self._step_count += 1

        # Publish an aciton, action = [linear_x, angular_z]
        # linear_x = action[0]
        # angular_z = action[1]
        linear_x = action[0] * 0.02
        angular_z = -2.84 + action[1] * 0.355
        self.ros_gz_interface.publish_twist([float(linear_x), float(angular_z)])

        # Spin once
        self.spin()

        # Get observation and info
        observation = self.get_observation()
        info = self.get_info()

        # Get the reward
        reward = self.compute_rewards(info)

        # Check if the turtlebot reaches the goal with a tolerence
        if (info["distance"] < self._tolerence):
            flag_1 = True
        else:
            flag_1 = False
        # flag_2 = self.ros_gz_interface.out_of_bound_penalty()
        # flag_3 = self.ros_gz_interface.obstacle_hit_penalty()
        self.done = flag_1
        terminated = self.done

        # truncated is also False
        truncated = self.truncated

        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        # Reset RL variables
        self.done = False
        self.laser_observation = np.array([np.float32(9)]*360)
        self.location_observation = np.array([0.0, 0.0])
        self.observation = np.append(self.laser_observation, self.location_observation)

        # self.ros_gz_interface = ros_gz_interface(self)

        # Spin once
        self.spin()

        # Get observation and info
        observation = self.get_observation()
        info = self.get_info()

        return observation, info

    def render(self):
        pass

    def close(self):
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

        if self.ros_gz_interface.obstacle_hit_penalty():
            reward += -1
            self.get_logger().info("Apply obstacle-hit penalty.")
        if self.ros_gz_interface.out_of_bound_penalty():
            reward += -3
            self.get_logger().info("Apply out-of-bound penalty.")

        return reward


def main(args=None):
    rclpy.init(args=args)
    simplified_env_node = simplified_env()
    rclpy.spin(simplified_env_node)
    simplified_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
