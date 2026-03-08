import time
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

        # Callback Group
        self.cbgroup = MutuallyExclusiveCallbackGroup()

        # Clients
        self.set_entity_pose_cli = self.create_client(
            SetEntityPose, '/world/empty/set_pose', callback_group=self.cbgroup
        )
        while not self.set_entity_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/world/empty/set_pose service unavailable')

        self._tolerence = 0.15
        self._max_translational_velocity = np.float32(0.22)
        self._max_rotational_vel = np.float32(2.84)
        self._min_detection_distance = np.float32(0.16)
        self._max_detection_distance = np.float32(8.0) # Not used

        # Action space: 
        # self.action_space = spaces.Box(
        #     low=np.array([-self._max_translational_velocity, -self._max_rotational_vel]),
        #     high=np.array([self._max_translational_velocity, self._max_rotational_vel]),
        #     dtype=np.float32
        # )
        self.action_space = spaces.Discrete(4, dtype=np.uint8)
        # Observation space: 
        obs_low = np.append(
            np.array([np.float32(0.06)]*36),
            np.array([np.float32(0), np.float32(-np.pi)])
        )
        obs_high = np.append(
            np.array([np.float32(9)]*36),
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
        self.reward = 0

        self._timestep_count = 0
        self._episode_count = 0
        self._reward_border = 1.0
        self._reward_count_1 = 0
        self._reward_count_2 = 0
        self._reward_count_3 = 0
        self._success_count = 0

        self.laser_observation = np.array([np.float32(9)]*36)
        self.location_observation = np.array([np.float32(8.4853), np.float32(np.pi)])
        self.observation = np.append(self.laser_observation, self.location_observation)

    def step(self, action):
        # Increment step count
        self._timestep_count += 1

        # Publish an aciton, action = [linear_x, angular_z]
        # linear_x = action[0]
        # angular_z = action[1]
        # self.ros_gz_interface.publish_twist([float(linear_x), float(angular_z)])
        if action == np.uint8(0):
            self.ros_gz_interface.publish_twist([0.20, 0.0])
        elif action == np.uint(1):
            self.ros_gz_interface.publish_twist([-0.10, 0.0])
        elif action == np.uint(2):
            self.ros_gz_interface.publish_twist([0.0, 1.57])
            time.sleep(1.0)
            self.spin()
            self.ros_gz_interface.publish_twist([0.20, 0.0])
        else:
            self.ros_gz_interface.publish_twist([0.0, -1.57])
            time.sleep(1.0)
            self.spin()
            self.ros_gz_interface.publish_twist([0.20, 0.0])

        # Spin once
        time.sleep(0.5)
        self.spin()

        # Get observation and info
        observation = self.get_observation()
        info = self.get_info()

        # Get the reward
        self.compute_rewards()
        reward = self.reward

        # Check if the turtlebot reaches the goal with a tolerence
        # After initialization, self.location_observation[0] will be very close to 0 briefly
        flag_1 = False
        if (self.location_observation[0] < self._tolerence) and self.location_observation[0] > 0.01:
            flag_1 = True
            self._success_count += 1
            self.get_logger().info(f"{self.location_observation[0]} \n")
        self.done = flag_1
        terminated = self.done

        # Check if the episode needs to be truncated
        flag_2 = False
        if self._timestep_count == 50:
            flag_2 = True
        self.truncated = flag_2
        truncated = self.truncated

        # # Debug
        # if self._timestep_count % 1000 == 0:
        #     self.get_logger().info(f"{self.ros_gz_interface._bearing}")

        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        self.get_logger().info("Reset...")

        # Reset RL variables
        self.done = False
        self.truncated = False
        self._timestep_count = 0
        self._reward_count_1 = 0
        self._reward_count_2 = 0
        self._reward_count_3 = 0

        self.laser_observation = np.array([np.float32(9)]*36)
        self.location_observation = np.array([np.float32(8.4853), np.float32(np.pi)])
        self.observation = np.append(self.laser_observation, self.location_observation)

        set_pose_request = SetEntityPose.Request()
        set_pose_request.entity.name = "turtlebot3_burger"
        set_pose_request.pose.position.x = 0.0
        set_pose_request.pose.position.y = 0.0
        set_pose_request.pose.position.z = 0.0
        set_pose_request.pose.orientation.z = 0.0
        success = False
        while not success:
            result = self.set_entity_pose_cli.call_async(set_pose_request)
            success = True
        self.get_logger().info("Turtlebot spawned at (0.0, 0.0), with an orientation of 0.0")

        # Reset the goal position
        # Skip for the first reset in each map configuration
        if self._episode_count != 0 and self.ros_gz_interface.obstacle_list_is_initialized():
            new_goal_pos = self.ros_gz_interface.reset_goal_position()
            self.get_logger().info(f"New goal position: ({new_goal_pos[0]}, {new_goal_pos[1]})")
        self._episode_count += 1

        # Spin once to get initial observation and info
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
        self.laser_observation = np.array([np.float32(9)]*36)
        self.location_observation = self.ros_gz_interface.get_distance_and_bearing()
        self.observation = np.append(self.laser_observation, self.location_observation)
        return self.observation

    def get_info(self):
        return {
            "distance": self.location_observation[0],
            "bearing": self.location_observation[1]
        }

    def compute_rewards(self):
        self.reward = 0

        # After initialization, self.location_observation[0] will be very close to 0 briefly
        if self.location_observation[0] < self._tolerence \
            and self.location_observation[0] > 0.01:
            # self.reward += 30
            self.get_logger().info(f"The turtlebot is within {self._tolerence} from the goal!")

        if self.location_observation[0] > 0.01:
            self.reward += 0.01*(1.0 - self.location_observation[0]/self._reward_border)

        # if self.ros_gz_interface.out_of_bound_penalty_grid():
        #     self.reward += -0.01
        #     # self.get_logger().info("Apply out-of-bound penalty (constant).")
        # if self.ros_gz_interface.obstacle_hit_penalty_grid():
        #     self.reward += -0.01
        #     # self.get_logger().info("Apply obstacle-hit penalty. (constant)")

        return self.reward


def main(args=None):
    rclpy.init(args=args)
    simplified_env_node = simplified_env()
    rclpy.spin(simplified_env_node)
    simplified_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
