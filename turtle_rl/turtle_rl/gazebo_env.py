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

        self.set_entity_pose_cli = self.create_client(
            SetEntityPose,
            '/world/empty/set_pose',
            callback_group=self._callback_group
        )

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

        # Services
        self.set_up_new_episode_ser = self.create_service(
            Empty,
            '/set_up_new_episode',
            callback=self.set_up_new_episode_callback
        )

        # Non-ROS variables
        self._worlds_path = "/home/zixin/ws/winter_project/src/turtle_rl/worlds"

    async def set_up_new_episode_callback(self, request, response):
        episode_num = 0

        ol, square_list, rl, cy, tp, tg= create_map(randomness=3)
        world_path = create_new_world(self._worlds_path, episode_num)

        square_count = 0
        for square in square_list:
            square_path = create_squares(square, square_count, world_path)
            request = SpawnEntity.Request()
            request.entity_factory.sdf_filename = square_path
            request.entity_factory.pose.position.x = square[0]
            request.entity_factory.pose.position.y = square[1]
            request.entity_factory.pose.position.z = 0.5
            request.entity_factory.pose.orientation.z = square[2]
            result = await self.spawn_entity_cli.call_async(request)

            square_count += 1

        return response


def main(args=None):
    rclpy.init(args=args)
    gazebo_env_node = gazebo_env()
    rclpy.spin(gazebo_env_node)
    gazebo_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
