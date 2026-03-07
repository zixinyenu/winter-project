import rclpy
import numpy as np
import gymnasium as gym
from rclpy.node import Node
from stable_baselines3 import PPO, TD3

from .ros_gz_interface import *
from .simplified_env import simplified_env

class displaying_node(Node):
    """gazebo_controller class."""

    def __init__(self):
        super().__init__('displaying_node')
        self.get_logger().info("The displaying node has just been created.")

        # Parameters
        self.declare_parameter('goal_x', -2.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('frequency', 100)
        self._goal_x = self.get_parameter('goal_x').value
        self._goal_y = self.get_parameter('goal_y').value
        self._fre = self.get_parameter('frequency').value

        # Subsciption
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            1
        )

        self.turtle_pos_sub = self.create_subscription(
            TFMessage,
            '/groundtruth_pose',
            self.turtle_pos_callback,
            1
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(
            1/self._fre,
            self.timer_callback
        )

        env = simplified_env()
        env.reset()
        models_dir = "/home/zixin/2025WINTER/ME499/winter_project_files/models/TD3"
        model_path = f"{models_dir}/14000000.zip"
        self.env = env
        self.model = TD3.load(model_path, env=env)

        self._turtle_pos = np.array([np.float32(0), np.float32(0)])
        self._turtle_ori = np.float32(0)
        self._laser_readings = np.array([np.float32(9)]*18) # LSD-02 can ony detect up to 8m, so 9m for inf
        self._distance = np.float32(0)
        self._bearing = np.float32(0)

    def timer_callback(self):
        turtle_x = self._turtle_pos[0]
        turtle_y = self._turtle_pos[1]
        goal_x = self._goal_x
        goal_y = self._goal_y
        distance = ((goal_x - turtle_x)**2 + (goal_y - turtle_y)**2)**0.5
        bearing = np.arctan2(
            turtle_y - goal_y,
            turtle_x - goal_x
        )
        self._distance = np.float32(distance)
        self._bearing = np.float32(bearing)

        # Get twist command from the model
        observation = self.get_obs()
        action, _ = self.model.predict(observation)
        observation, reward, terminated, truncated, info = self.env.step(action)
        # Publish the twist
        msg = Twist()
        msg.linear.x = float(action[0])
        msg.angular.z = float(action[1])
        self.cmd_vel_pub.publish(msg)

    def get_obs(self):
        observation = np.append(
            self._laser_readings,
            np.array([self._distance, self._bearing])
        )
        return observation

    def laser_callback(self, msg: LaserScan):
        laser_reading = np.array(msg.ranges)
        # Convert infinity to 9m, since LSD-02 can only detect up to 8m
        laser_reading[laser_reading == np.inf] = np.float32(9)
        laser_reading[laser_reading == -np.inf] = np.float32(0.06)
        self._laser_readings = laser_reading

    def turtle_pos_callback(self, msg: TFMessage):
        # /groundtruth_pose contains TFMessage type message
        # it is like a pose vector or pose list
        # the one with child_frame_id: turtlebot3_burger is groundtruth pose of the turtlebot3
        for tf in msg.transforms:
            if tf.child_frame_id == "turtlebot3_burger":
                self._turtle_pos = np.array([
                    np.float32(tf.transform.translation.x),
                    np.float32(tf.transform.translation.y)
                ])
                angles = euler_from_quaternion([
                    tf.transform.rotation.w,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z
                ])
                self._turtle_ori = angles[2]
        # self.node.get_logger().info(f'turtle pos: {self._turtle_pos[0]}, {self._turtle_pos[1]}')
        # self.node.get_logger().info(f'turtle ori: {self._turtle_ori}')

def main(args=None):
    rclpy.init(args=args)
    display_node = displaying_node()
    rclpy.spin(display_node)
    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
