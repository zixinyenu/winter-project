import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose, SpawnEntity, DeleteEntity

from .map_env import *

import numpy as np

class gazebo_env(Node):
    """gazebo_env class."""

    def __init__(self):
        super().__init__('gazebo_env')
        self.get_logger().info("The gazebo_env node has just been created.")

        # Clients
        self._callback_group = MutuallyExclusiveCallbackGroup()

        self.control_world_cli = self.create_client(
            ControlWorld,
            '/world/empty/control',
            callback_group=self._callback_group
        )
        while not self.control_world_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set control world client to be available...")

        self.set_entity_pose_cli = self.create_client(
            SetEntityPose,
            '/world/empty/set_pose',
            callback_group=self._callback_group
        )
        while not self.set_entity_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set entity pose client to be available...")

        self.spawn_entity_cli = self.create_client(
            SpawnEntity,
            '/world/empty/create',
            callback_group=self._callback_group
        )
        while not self.spawn_entity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn entity client to be available...")

        self.delete_entity_cli = self.create_client(
            DeleteEntity,
            '/world/empty/remove',
            callback_group=self._callback_group
        )
        while not self.delete_entity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for delete entity client to be available...")

        # Services
        self.control_simulation_ser = self.create_service(
            Empty,
            '/control_simulation',
            callback=self.control_simulation_callback
        )
        self.set_up_new_episode_ser = self.create_service(
            Empty,
            '/set_up_new_episode',
            callback=self.set_up_new_episode_callback
        )
        self.delete_all_obstacles_ser = self.create_service(
            Empty,
            '/delete_all_obstacles',
            callback=self.delete_all_obstacles_callback
        )

        # Non-ROS variables
        self._simulation_paused = False
        self._worlds_path = "/home/zixin/ws/winter_project/src/turtle_rl/worlds"
        self._square_count = -1
        self._rectangle_count = -1
        self._cylinder_count = -1
        self._goal_pos = [8, 8, 0.0]
        self._episode_num = 0

    async def control_simulation_callback(self, request, response):
        control_world_request = ControlWorld.Request()
        if self._simulation_paused:
            self._simulation_paused = False
            control_world_request.world_control.pause = False
            self.get_logger().info("Gazebo simulation has been resumed.")
        else:
            self._simulation_paused = True
            control_world_request.world_control.pause = True
            self.get_logger().info("Gazebo simulation has been paused.")
        result = await self.control_world_cli.call_async(control_world_request)

        return response

    async def set_up_new_episode_callback(self, request, response):
        episode_num = self._episode_num
        world_path = create_new_world(self._worlds_path, episode_num)

        ol, square_list, rectangle_list, cylinder_list, start_pos, goal_pos= create_map(randomness=3)
        self._goal_pos = goal_pos
        self.get_logger().info(f"Goal position set at ({goal_pos[0]}, {goal_pos[1]})")

        set_pose_request = SetEntityPose.Request()
        set_pose_request.entity.name = "turtlebot3_burger"
        set_pose_request.pose.position.x = start_pos[0]
        set_pose_request.pose.position.y = start_pos[1]
        set_pose_request.pose.position.z = 1.0
        set_pose_request.pose.orientation.z = start_pos[2]
        result = await self.set_entity_pose_cli.call_async(set_pose_request)
        self.get_logger().info(f"Turtlebot spawned at ({start_pos[0]}, {start_pos[1]}), with an orientation of {start_pos[2]}")

        square_count = 0
        for square in square_list:
            square_path = create_squares(square, square_count, world_path)
            spawn_request = SpawnEntity.Request()
            spawn_request.entity_factory.sdf_filename = square_path
            spawn_request.entity_factory.pose.position.x = square[0]
            spawn_request.entity_factory.pose.position.y = square[1]
            spawn_request.entity_factory.pose.position.z = 0.5
            spawn_request.entity_factory.pose.orientation.z = square[2]
            result = await self.spawn_entity_cli.call_async(spawn_request)
            square_count += 1
        self.get_logger().info("Squares all created successfully.")
        self._square_count = square_count

        rectangle_count = 0
        for rectangle in rectangle_list:
            rectangle_path = create_rectangles(rectangle, rectangle_count, world_path)
            spawn_request = SpawnEntity.Request()
            spawn_request.entity_factory.sdf_filename = rectangle_path
            spawn_request.entity_factory.pose.position.x = rectangle[0]
            spawn_request.entity_factory.pose.position.y = rectangle[1]
            spawn_request.entity_factory.pose.position.z = 0.5
            spawn_request.entity_factory.pose.orientation.z = rectangle[2]
            result = await self.spawn_entity_cli.call_async(spawn_request)
            rectangle_count += 1
        self.get_logger().info("Rectangles all created successfully.")
        self._rectangle_count = rectangle_count

        cylinder_count = 0
        for cylinder in cylinder_list:
            cylinder_path = create_cylinders(cylinder, cylinder_count, world_path)
            spawn_request = SpawnEntity.Request()
            spawn_request.entity_factory.sdf_filename = cylinder_path
            spawn_request.entity_factory.pose.position.x = cylinder[0]
            spawn_request.entity_factory.pose.position.y = cylinder[1]
            spawn_request.entity_factory.pose.position.z = 0.5
            result = await self.spawn_entity_cli.call_async(spawn_request)
            cylinder_count += 1
        self.get_logger().info("Cylinders all created successfully.")
        self._cylinder_count = cylinder_count

        return response

    async def delete_all_obstacles_callback(self, request, response):
        delete_request = DeleteEntity.Request()

        # square/rectangle: type 2
        # cylinder: type 2
        for square_num in range(self._square_count):
            square_name = f'square_{square_num}'
            delete_request.entity.name = square_name
            delete_request.entity.type = 2
            result = await self.delete_entity_cli.call_async(delete_request)
        self.get_logger().info("Squares all deleted successfully.")
        self._square_count = 0

        for rectangle_num in range(self._rectangle_count):
            rectangle_name = f'rectangle_{rectangle_num}'
            delete_request.entity.name = rectangle_name
            delete_request.entity.type = 2
            result = await self.delete_entity_cli.call_async(delete_request)
        self.get_logger().info("Rectangles all deleted successfully.")
        self._rectangle_count = 0

        for cylinder_num in range(self._cylinder_count):
            cylinder_name = f'cylinder_{cylinder_num}'
            delete_request.entity.name = cylinder_name
            delete_request.entity.type = 2
            result = await self.delete_entity_cli.call_async(delete_request)
        self.get_logger().info("Cylinders all deleted successfully.")
        self._cylinder_count = 0

        return response


def main(args=None):
    rclpy.init(args=args)
    gazebo_env_node = gazebo_env()
    rclpy.spin(gazebo_env_node)
    gazebo_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
