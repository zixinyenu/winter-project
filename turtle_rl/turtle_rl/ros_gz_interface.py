from rclpy.node import Node

from .turtle_env import *
from .gazebo_env import *

class ros_gz_interface(Node):
    """ros_gz_interface class."""

    def __init__(self):
        super().__init__('ros_gz_interface')
        # self.node = node
        self.turtle_environment = turtle_env(self)
        self.gazebo_env = gazebo_env(self)

def main(args=None):
    rclpy.init(args=args)
    interface_node = ros_gz_interface()
    rclpy.spin(interface_node)
    interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
