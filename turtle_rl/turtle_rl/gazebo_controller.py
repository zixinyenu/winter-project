import rclpy
from rclpy.node import Node
from example_interfaces.msg import UInt8MultiArray, Float32MultiArray

from .gazebo_env import gazebo_env

class gazebo_controller(Node):
    """gazebo_controller class."""

    def __init__(self):
        super().__init__('gazebo_controller')
        self.get_logger().info("The gazebo controller node has just been created.")

        self.gazebo_controller = gazebo_env(self)

        # Publishers
        self.obstacle_list_pub = self.create_publisher(
            UInt8MultiArray,
            '/obstacle_list',
            10
        )
        self.goal_pos_pub = self.create_publisher(
            Float32MultiArray,
            '/goal_pos',
            10
        )

        # Timer
        self.timer = self.create_timer(
            1,
            self.timer_callback
        )

    def timer_callback(self):
        msg_ol = UInt8MultiArray()
        msg_ol.data = self.gazebo_controller._obstacle_list
        self.obstacle_list_pub.publish(msg_ol)

        msg_gp = Float32MultiArray()
        msg_gp.data = self.gazebo_controller._goal_pos
        self.goal_pos_pub.publish(msg_gp)


def main(args=None):
    rclpy.init(args=args)
    controller_node = gazebo_controller()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
