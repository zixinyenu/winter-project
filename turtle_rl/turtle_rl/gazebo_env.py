import rclpy
from rclpy.node import Node

import numpy as np

class gazebo_env(Node):
    """gazebo_env class."""

    def __init__(self):
        super().__init__('gazebo_env')
        self.get_logger().info("The gazebo_env node has just been created.")


def main(args=None):
    rclpy.init(args=args)
    gazebo_env_node = gazebo_env()
    rclpy.spin(gazebo_env_node)
    gazebo_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
