from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

import rclpy
import numpy as np
from rclpy.node import Node
from .astarInterface import *


def distance_between_two_points(p1, p2):
    return pow(pow((p1[0] - p2[0]), 2) + pow((p1[1] - p2[1]), 2), 0.5)

class turtleControl(Node):
    """turtleControl class."""

    def __init__(self):
        super().__init__('turtleControl')

        # Parameters
        self.declare_parameter('cmd_fre', 1)

        # Private Variables
        self._cmd_fre = self.get_parameter('cmd_fre').value
        self._start_pst = [1.0, 1.0]
        self._goal_pst = [3.0, 3.0]
        self._xlist, self._ylist = run_astar(self._start_pst, self._goal_pst, online=False)
        self._waypoint_count = 0

        self._curr_pst = self._start_pst
        self._curr_ori = [0.0, 0.0, 0.0, 1.0]

        # Subscription
        self.curr_pst_subscription = self.create_subscription(Odometry, 
                                                              '/odom', 
                                                              self.turtle_position_callback, 
                                                              10)
        self.curr_ori_subscription = self.create_subscription(Odometry, 
                                                              '/odom', 
                                                              self.turtle_orientation_callback, 
                                                              10)

        # Publishers
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.twist_timer_ = self.create_timer(self._cmd_fre, self.twist_callback)

    def turtle_position_callback(self, msg):
        self._curr_pst = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def turtle_orientation_callback(self, msg):
        self._curr_ori = [
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        ]

    def twist_callback(self):
        """Publish velocity command to the robot."""
        curr_pst = self._curr_pst
        if distance_between_two_points(curr_pst, self._goal_pst) > 0.15:
            self.get_logger().info("Goal Not Reached!")
            next_waypoint = [
                self._xlist[self._waypoint_count], 
                self._ylist[self._waypoint_count]
            ]
            if distance_between_two_points(curr_pst, next_waypoint) > 0.05:
                # Calculate the angle to the waypoint
                theta = np.arctan2(
                    self._goal_pst[1] - curr_pst[1], 
                    self._goal_pst[0] - curr_pst[0]
                )
                curr_ori = self._curr_ori
                euler = euler_from_quaternion(curr_ori)
                yaw = euler[2]
                angular_velocity = 0.3*(theta-yaw) # P controller for angular velocity

                # Create Twist message to be published onto /cmd_vel
                twist = Twist()
                twist.linear.x = 0.15
                twist.angular.z = angular_velocity

                # Call the publisher
                self.twist_publisher_.publish(twist)
            else:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.twist_publisher_.publish(twist)
                # Increment the counter
                self._waypoint_count += 1
        else:
            self.get_logger().info(f"Goal Reached! {curr_pst}")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twist_publisher_.publish(twist)
        

def main(args=None):
    rclpy.init(args=args)
    turtleControl_node = turtleControl()
    rclpy.spin(turtleControl_node)
    turtleControl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()