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

        # Gazebo factor
        self.declare_parameter("real_time_factor", 10.0)
        self._real_time_factor = self.get_parameter("real_time_factor").value

        # Callback Group
        self.cbgroup = MutuallyExclusiveCallbackGroup()

        # Clients
        self.set_entity_pose_cli = self.create_client(
            SetEntityPose, '/world/empty/set_pose', callback_group=self.cbgroup
        )
        while not self.set_entity_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/world/empty/set_pose service unavailable')

        self._tolerence = 0.25
        self._max_translational_velocity = np.float32(0.22)
        self._max_rotational_vel = np.float32(2.84)
        self._min_detection_distance = np.float32(0.16)
        self._max_detection_distance = np.float32(8.0) # Not used

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
        self._reward_border = 1.50
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

        if action == np.uint8(0):
            self.ros_gz_interface.publish_twist([0.20, 0.0])
            time.sleep(0.50/self._real_time_factor)
        elif action == np.uint(1):
            self.ros_gz_interface.publish_twist([-0.10, 0.0])
            time.sleep(0.50/self._real_time_factor)
        elif action == np.uint(2):
            self.ros_gz_interface.publish_twist([0.0, np.pi/2.0])
            time.sleep(1.0/self._real_time_factor)
            self.spin()
            self.ros_gz_interface.publish_twist([0.18, 0.0])
            time.sleep(1.00/self._real_time_factor)
        else:
            self.ros_gz_interface.publish_twist([0.0, -np.pi/2.0])
            time.sleep(1.0/self._real_time_factor)
            self.spin()
            self.ros_gz_interface.publish_twist([0.18, 0.0])
            time.sleep(1.00/self._real_time_factor)

        # Spin once
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
        if self._timestep_count == 100:
            flag_2 = True
        self.truncated = flag_2
        truncated = self.truncated

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

        set_pose_request_1 = SetEntityPose.Request()
        set_pose_request_1.entity.name = "turtlebot3_burger"
        set_pose_request_1.pose.position.x = 0.0
        set_pose_request_1.pose.position.y = 0.0
        set_pose_request_1.pose.position.z = 0.0
        set_pose_request_1.pose.orientation.z = 0.0
        success = False
        while not success:
            result = self.set_entity_pose_cli.call_async(set_pose_request_1)
            success = True
        self.get_logger().info("Turtlebot spawned at (0.0, 0.0), with an orientation of 0.0")

        # Reset the goal position
        # Skip for the first reset in each map configuration
        if self._episode_count != 0 and self.ros_gz_interface.obstacle_list_is_initialized():
            new_goal_pos = self.ros_gz_interface.reset_goal_position_p13()
            set_pose_request_2 = SetEntityPose.Request()
            set_pose_request_2.entity.name = "goal_visual"
            set_pose_request_2.pose.position.x = new_goal_pos[0]
            set_pose_request_2.pose.position.y = new_goal_pos[1]
            set_pose_request_2.pose.position.z = 0.0
            success = False
            while not success:
                result = self.set_entity_pose_cli.call_async(set_pose_request_2)
                success = True
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
            self.reward += 100
            self.get_logger().info(f"The turtlebot is within {self._tolerence} from the goal!")

        if self.location_observation[0] > 0.01:
            self.reward += 10.0*(1.0 - self.location_observation[0]/self._reward_border)

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
