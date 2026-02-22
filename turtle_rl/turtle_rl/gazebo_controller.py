import rclpy
from rclpy.node import Node

from .ros_gz_interface import *

class gazebo_controller(Node):
    """gazebo_controller class."""

    def __init__(self):
        super().__init__('gazebo_controller')
        self.get_logger().info("The gazebo controller node has just been created.")

        self.ros_gz_interface = ros_gz_interface(self)

def main(args=None):
    rclpy.init(args=args)
    controller_node = gazebo_controller()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
