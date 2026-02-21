import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from tf_transformations import euler_from_quaternion

class turtle_env:
    """turtle_env class."""

    def __init__(self, node: Node):
        # super().__init__('turtle_env')
        self.node = node
        self.node.get_logger().info("The turtle_env fraction has just been created.")

        # Parameter
        try:
            self.node.declare_parameter('frequency', 100)
        except ParameterAlreadyDeclaredException:
            self.node.get_logger().info("Parameter \"frequency\" has already been declared in previous step.")
        self.fre_ = self.node.get_parameter('frequency').value

        # Subscription
        self.laser_sub = self.node.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            1
        )
        self.turtle_pos_sub = self.node.create_subscription(
            TFMessage,
            '/groundtruth_pose',
            self.turtle_pos_callback,
            1
        )
        self.fence_contact_sub = self.node.create_subscription(
            Bool,
            '/fence/touched',
            self.fence_contact_callback,
            1
        )
        self.obstacle_contact_sub = self.node.create_subscription(
            Bool,
            '/obstacle/touched',
            self.obstacle_contact_callback,
            1
        )

        # Publisher
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.contact_timer =  self.node.create_timer(
            1/self.fre_,
            self.timer_callback
        )

        # Class variables
        self._laser_readings = np.array([np.float32(9)]*360) # LSD-02 can ony detect up to 8m, so 9m for inf
        self._turtle_pos = np.array([np.float32(0), np.float32(0)])
        self._turtle_ori = np.float32(0)
        self._prev_fence_contact_count = 0
        self._curr_fence_contact_count = 0
        self._out_of_bound_penalty = False
        self._prev_obstacle_contact_count = 0
        self._curr_obstacle_contact_count = 0
        self._obstacle_hit_penalty = False

    def timer_callback(self):
        if self._prev_fence_contact_count != self._curr_fence_contact_count:
            self._out_of_bound_penalty = True
            self.node.get_logger().info(f"Apply out-of-bound penalty: {self._out_of_bound_penalty}")
            self._prev_fence_contact_count = self._curr_fence_contact_count
        else:
            self._out_of_bound_penalty = False

        if self._prev_obstacle_contact_count != self._curr_obstacle_contact_count:
            self._obstacle_hit_penalty = True
            self.node.get_logger().info(f"Apply obstacle-hit penalty: {self._obstacle_hit_penalty}")
            self._prev_obstacle_contact_count = self._curr_obstacle_contact_count
        else:
            self._obstacle_hit_penalty = False

    def fence_contact_callback(self, msg: Bool):
        if msg.data is True:
            self._curr_fence_contact_count += 1
        # self.node.get_logger().info(f"Previous fence contact count: {self._prev_fence_contact_count}")
        # self.node.get_logger().info(f"Current fence contact count: {self._curr_fence_contact_count}")

    def obstacle_contact_callback(self, msg: Bool):
        if msg.data is True:
            self._curr_obstacle_contact_count += 1
        # self.node.get_logger().info(f"Previous obstacle contact count: {self._prev_obstacle_contact_count}")
        # self.node.get_logger().info(f"Current obstacle contact count: {self._curr_obstacle_contact_count}")

    def laser_callback(self, msg: LaserScan):
        laser_reading = np.array(msg.ranges)
        # Convert infinity to 9m, since LSD-02 can only detect up to 8m
        laser_reading[laser_reading == np.inf] = np.float32(9)
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

    def cmd_vel_publish(self, vel_list):
        msg = Twist()
        msg.linear.x = vel_list[0]
        msg.angular.z = vel_list[1]
        self.cmd_vel_pub.publish(msg)

    def turtle_env_destroy(self):
        self.node.destroy_subscription(self.laser_sub)
        self.node.destroy_subscription(self.turtle_pos_sub)
        self.node.destroy_subscription(self.fence_contact_sub)
        self.node.destroy_subscription(self.obstacle_contact_sub)
        self.node.destroy_publisher(self.cmd_vel_pub)
        self.node.destroy_timer(self.contact_timer)


# def main(args=None):
#     rclpy.init(args=args)
#     turtle_env_node = turtle_env()
#     rclpy.spin(turtle_env_node)
#     turtle_env_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
