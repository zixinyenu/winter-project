"""
Control a robot to move back and forth.

PUBLISHERS:
  + /cmd_vel (Twist) - The velocity of the moving robot

PARAMETERS:
  + cmd_fre (Parameter/Type/FLOAT) - Frequency of sending velocity command to the robot
"""

from geometry_msgs.msg import Twist, Vector3

import rclpy
from rclpy.node import Node


class moveTest(Node):
    """moveTest class."""

    def __init__(self):
        super().__init__('moveTest')

        # Parameters
        self.declare_parameter('cmd_fre', 1)

        # Private Variables
        self._cmd_fre = self.get_parameter('cmd_fre').value
        self._is_turning = False
        self._turn_sign = 1
        self._turn_counter = 0
        self._x_vel = 5.0

        # Publishers
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.twist_timer_ = self.create_timer(self._cmd_fre, self.twist_callback)

    def twist_callback(self):
        """Publish velocity command to the robot."""
        if self._is_turning:
            twist = self.get_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.twist_publisher_.publish(twist)
            self._turn_counter = 0
            self._is_turning = False
        else:
            x_vel = self._turn_sign*self._x_vel
            twist = self.get_twist([x_vel, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.twist_publisher_.publish(twist)
            self._turn_counter += 1
            if self._turn_counter == 3:
                self._turn_sign = -1 * self._turn_sign
                self._is_turning = True

    def get_twist(self, linear_vel, angular_vel):
        """
        Convert linear and angular velocity data into geometry_msgs/msg/Twist type variable.

        linear_vel : float[]
            A list contains the linear velocities in x-y-z directions

        angular_val : float[]
            A list contains the angular velocities along x-y-z directions
        """
        return Twist(
            linear=Vector3(
                x=linear_vel[0], y=linear_vel[1], z=linear_vel[2]
            ),
            angular=Vector3(
                x=angular_vel[0], y=angular_vel[1], z=angular_vel[2]
            )
        )


def main(args=None):
    rclpy.init(args=args)
    moveTest_node = moveTest()
    rclpy.spin(moveTest_node)
    moveTest_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()