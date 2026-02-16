import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

import numpy as np

class turtle_env(Node):
    """TurtleEnv class."""

    def __init__(self):
        super().__init__('turtle_env')
        self.get_logger().info("The turtle_env node has just been created.")

        # Parameter

        # Subscription
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 1)
        self.turtle_pos_sub = self.create_subscription(Odometry, 'odom', self.turtle_pos_callback, 1)

        # Class variables
        self._laser_readings = np.array([np.float32(10)]*360) # LSD-02 can ony detect up to 8m, so 10m for inf
        self._turtle_pos = np.array([np.float32(0), np.float32(0)])
        self._turtle_ori = np.float32(0)

    def laser_callback(self, msg: LaserScan):
        self._laser_readings = np.array(msg.ranges)
        # Convert infinity to 10m, since LSD-02 can only detect up to 8m
        self._laser_readings[self._laser_readings == np.inf] =np.float32(10)

    def turtle_pos_callback(self, msg: Odometry):
        # clip?
        self._turtle_pos = np.array([
            np.float32(msg.pose.pose.position.x),
            np.float32(msg.pose.pose.position.y)
        ])
        angles = euler_from_quaternion([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])
        self._turtle_ori = angles[2]


def main(args=None):
    rclpy.init(args=args)
    turtle_env_node = turtle_env()
    rclpy.spin(turtle_env_node)
    turtle_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
