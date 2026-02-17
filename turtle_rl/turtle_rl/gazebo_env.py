import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose, SpawnEntity, DeleteEntity

import numpy as np

class gazebo_env(Node):
    """gazebo_env class."""

    def __init__(self):
        super().__init__('gazebo_env')
        self.get_logger().info("The gazebo_env node has just been created.")

        # Client
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

        self.delete_entity_cli = self.create_client(
            DeleteEntity,
            '/world/empty/remove',
            callback_group=self._callback_group
        )


def main(args=None):
    rclpy.init(args=args)
    gazebo_env_node = gazebo_env()
    rclpy.spin(gazebo_env_node)
    gazebo_env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
