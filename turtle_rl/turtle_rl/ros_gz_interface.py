import numpy as np

from rclpy.node import Node
from example_interfaces.msg import UInt8MultiArray, Float32MultiArray

from .turtle_env import *
from .gazebo_env import *

class ros_gz_interface:
    """ros_gz_interface class."""

    def __init__(self, node: Node):
        # super().__init__('ros_gz_interface')
        self.node = node
        self.turtle_environment = turtle_env(self.node)

        # Subscriptions
        self.obstacle_list_sub = self.node.create_subscription(
            UInt8MultiArray,
            '/obstacle_list',
            self.obstacle_list_callback,
            1
        )
        self.goal_pos_sub = self.node.create_subscription(
            Float32MultiArray,
            '/goal_pos',
            self.goal_pos_callback,
            1
        )

        # Timer
        self.main_timer = self.node.create_timer(
            1/self.turtle_environment.fre_,
            self.timer_callback
        )

        # Non-ROS variables
        self._distance = np.float32(0)
        self._bearing = np.float32(0)
        self._out_of_bound_grid = False
        self._obstacle_hit_grid = False
        self._obstacle_list = []
        self._goal_pos = [0.0, 0.0]

    ########## ROS_Functions_Start ##########
    # action = [linear_x, angular_z]
    def publish_twist(self, action):
        self.turtle_environment.cmd_vel_publish(action)

    def get_distance_and_bearing(self):
        return np.array([self._distance, self._bearing])

    def get_laser_readings(self):
        return self.turtle_environment._laser_readings

    def out_of_bound_penalty_init(self):
        return self.turtle_environment._out_of_bound_penalty

    def obstacle_hit_penalty_init(self):
        return self.turtle_environment._obstacle_hit_penalty

    def out_of_bound_penalty_grid(self):
        return self._out_of_bound_grid

    def obstacle_hit_penalty_grid(self):
        return self._obstacle_hit_grid

    def interface_destroy(self):
        self.turtle_environment.turtle_env_destroy()

    # Genius design
    def reset_goal_position(self, border):
        success_1 = False
        success_2 = False
        success_3 = False
        while not success_3:
            x, y, rad, radius, pending_list, success_1 = gen_turtle_apt(map_length=6, map_width=6, d=100, collision_radius=0.11)
            if success_1 == False:
                continue
            success_2 = check_turtle_collision(self._obstacle_list, pending_list)
            if success_2 == False:
                continue
            turtle_x = self.turtle_environment._turtle_pos[0]
            turtle_y = self.turtle_environment._turtle_pos[1]
            success_3 = ((turtle_x - x)**2 + (turtle_y - y)**2)**0.5 > border
        self._goal_pos = [x, y]
        return self._goal_pos

    def obstacle_list_is_initialized(self):
        obstacle_list = self._obstacle_list
        # When the map has not been initialized, obstacle_list has a size of 1
        # Empty list is not used since it causes error elsewhere
        if len(obstacle_list) == 1:
            return False
        else:
            return True

    def obstacle_list_callback(self, msg: UInt8MultiArray):
        tmp_obstacle_list = msg.data
        obstacle_list = np.zeros(shape=(601, 601), dtype=np.uint8)
        for idx, i in enumerate(tmp_obstacle_list):
            obstacle_list[int(idx/601)][idx%601] = i
        self._obstacle_list = obstacle_list

    def goal_pos_callback(self, msg: UInt8MultiArray):
        self._goal_pos = msg.data
    ########## ROS_Functions_End ##########

    def timer_callback(self):
        self._distance, self._bearing = self._get_distance_and_bearing()
        # self.node.get_logger().info(f"Turtlebot->Goal    distance: {self._distance}, bearing: {self._bearing}")

        self._out_of_bound_grid = self._out_of_bound_grid_check()

        obstacle_list = self._obstacle_list
        turtle_pending_list = self._create_turtle_pending_list()
        if not self._out_of_bound_grid:
            try:
                self._obstacle_hit_grid = check_turtle_collision(obstacle_list, turtle_pending_list)
            except IndexError as e:
                pass
                # self.node.get_logger().info(f"{e} - Ignore if this appears when the map has not been set up.")

    ########## Helper_Functions_Start ##########
    def _get_distance_and_bearing(self):
        turtle_x = self.turtle_environment._turtle_pos[0]
        turtle_y = self.turtle_environment._turtle_pos[1]
        goal_x = self._goal_pos[0]
        goal_y = self._goal_pos[1]
        distance = ((goal_x - turtle_x)**2 + (goal_y - turtle_y)**2)**0.5
        bearing = np.arctan2(
            turtle_y - goal_y,
            turtle_x - goal_x
        )
        return np.float32(distance), np.float32(bearing)

    def _out_of_bound_grid_check(self, map_length=6, map_width=6, collision_raidus=0.11):
        turtle_x = self.turtle_environment._turtle_pos[0]
        turtle_y = self.turtle_environment._turtle_pos[1]
        if abs(turtle_x)+collision_raidus >= map_length/2 or abs(turtle_y)+collision_raidus >= map_width/2:
            return True
        else:
            return False

    def _create_turtle_pending_list(self, map_length=6, map_width=6, division=100, collision_radius=0.11):
        turtle_x = self.turtle_environment._turtle_pos[0]
        turtle_y = self.turtle_environment._turtle_pos[1]

        pending_list = []

        ystart = turtle_y-collision_radius
        xstart = turtle_x-collision_radius
        ystop = (turtle_y+collision_radius)+(1/division)
        xstop = (turtle_x+collision_radius)+(1/division)

        for yp in np.arange(start=ystart, stop=ystop, step=1/division):
            for xp in np.arange(start=xstart, stop=xstop, step=1/division):
                if (yp - turtle_y)**2 + (xp-turtle_x)**2 > collision_radius**2:
                    continue
                i, j = xy2ij(xp, yp, -map_length/2, -map_width/2, division)
                pending_list.append([i, j])
        return pending_list
    ########## Helper_Functions_End ##########

# def main(args=None):
#     rclpy.init(args=args)
#     interface_node = ros_gz_interface()
#     rclpy.spin(interface_node)
#     interface_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
