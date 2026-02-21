import numpy as np

from rclpy.node import Node

from .turtle_env import *
from .gazebo_env import *

class ros_gz_interface:
    """ros_gz_interface class."""

    def __init__(self, node: Node):
        # super().__init__('ros_gz_interface')
        self.node = node
        self.turtle_environment = turtle_env(self.node)
        self.gazebo_environment = gazebo_env(self.node)

        # Timer
        self.main_timer = self.node.create_timer(
            1/self.turtle_environment.fre_,
            self.timer_callback
        )

        # Non-ROS variables
        self.distance = 0
        self.bearing = 0

    ########## ROS_Functions_Start ##########
    # action = [linear_x, angular_z]
    def publish_twist(self, action):
        self.turtle_environment.cmd_vel_publish(action)

    def get_distance_and_bearing(self):
        return self.distance, self.bearing

    def get_laser_readings(self):
        return self.turtle_environment._laser_readings

    def out_of_bound_penalty(self):
        return self.turtle_environment._out_of_bound_penalty

    def obstacle_hit_penalty(self):
        return self.turtle_environment._obstacle_hit_penalty
    ########## ROS_Functions_End ##########

    def set_episode_num(self, episode_num):
        self.gazebo_environment.episode_num_setter(episode_num)

    def timer_callback(self):
        self.distance, self.bearing = self._get_distance_and_bearing()
        # self.node.get_logger().info(f"Turtlebot->Goal    distance: {self.distance}, bearing: {self.bearing}")

    ########## Helper_Functions_Start ##########
    def _get_distance_and_bearing(self):
        turtle_x = self.turtle_environment._turtle_pos[0]
        turtle_y = self.turtle_environment._turtle_pos[1]
        goal_x = self.gazebo_environment._goal_pos[0]
        goal_y = self.gazebo_environment._goal_pos[1]
        distance = ((goal_x - turtle_x)**2 + (goal_y - turtle_y)**2)**0.5
        bearing = np.arctan2(
            turtle_y - goal_y,
            turtle_x - goal_x
        )
        return distance, bearing
    ########## Helper_Functions_End ##########

# def main(args=None):
#     rclpy.init(args=args)
#     interface_node = ros_gz_interface()
#     rclpy.spin(interface_node)
#     interface_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
