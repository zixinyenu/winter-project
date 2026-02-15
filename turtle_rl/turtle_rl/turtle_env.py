import rclpy
from rclpy.node import Node 

class turtle_env(Node):
    """TurtleEnv class."""

    def __init__(self):
        super().__init__('turtle_env')
        self.get_logger().info("The turtle_env node has just been created.")

def main(args=None):
    rclpy.init(args=args)
    turtle_env_node = turtle_env()
    rclpy.spin(turtle_env_node)
    turtle_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
