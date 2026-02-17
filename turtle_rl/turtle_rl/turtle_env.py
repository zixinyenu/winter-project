import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

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
        self.turtle_pos_sub = self.create_subscription(TFMessage, '/groundtruth_pose', self.turtle_pos_callback, 1)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Class variables
        self._laser_readings = np.array([np.float32(10)]*360) # LSD-02 can ony detect up to 8m, so 10m for inf
        self._turtle_pos = np.array([np.float32(0), np.float32(0)])
        self._turtle_ori = np.float32(0)

    def cmd_vel_publish(self, vel_list):
        msg = Twist()
        msg.linear.x = vel_list[0]
        msg.angular.z = vel_list[1]
        self.cmd_vel_pub.publish(msg)

    def laser_callback(self, msg: LaserScan):
        laser_reading = np.array(msg.ranges)
        # Convert infinity to 10m, since LSD-02 can only detect up to 8m
        laser_reading[laser_reading == np.inf] = np.float32(10)
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
        # self.get_logger().info(f'turtle pos: {self._turtle_pos[0]}, {self._turtle_pos[1]}')
        # self.get_logger().info(f'turtle ori: {self._turtle_ori}')


def main(args=None):
    rclpy.init(args=args)
    turtle_env_node = turtle_env()
    rclpy.spin(turtle_env_node)
    turtle_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
