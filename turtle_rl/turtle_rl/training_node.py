import rclpy
import gymnasium as gym
from rclpy.node import Node
from gymnasium.envs import registration
from stable_baselines3 import PPO, TD3
from stable_baselines3.common import env_checker
from stable_baselines3.common import callbacks
from stable_baselines3.common.monitor import Monitor
# from stable_baselines3.common.vec_env import DummyVecEnv, VecCheckNan

from .simplified_env import simplified_env

import os

class training_node(Node):
    """training_node class."""

    def __init__(self):
        super().__init__('training_node')
        self.get_logger().info("The training node has just been created.")

        # Parameters
        self.declare_parameter('training_mode', 'retrain')
        self.declare_parameter('epoch', 0)
        self._training_mode = self.get_parameter('training_mode').value
        self._epoch = self.get_parameter('epoch').value

def main(args=None):
    # Initialize training node
    rclpy.init(args=args)
    node = training_node()

    # Create directories where the trained RL models and logswill be saved
    algorithm = "PPO"
    models_dir = f"/home/zixin/ws/winter_project/src/turtle_rl/models"
    logs_dir = f"/home/zixin/ws/winter_project/src/turtle_rl/logs"

    # Create directories for models and logs if they have not been created
    if not os.path.exists(models_dir):
        os.makedirs(models_dir)
    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)

    env = simplified_env()
    env = Monitor(env)

        # Check not a number errors
    # env = DummyVecEnv([lambda: simplified_env()])
    # env = VecCheckNan(env, raise_exception=True)
        # Check custom environment to see if it is fine
    # env_checker.check_env(env)
    # node.get_logger().info("The simplified_env has been checked.")

    # Reset the environment first
    env.reset()

    # Training!
    if node._training_mode == 'train':
        try:
            model = PPO(
                policy="MlpPolicy",
                env=env,
                verbose=1,
                tensorboard_log=logs_dir,
                learning_rate=0.0003,
                n_steps=2048
            )

            TIMESTEPS = 10000
            for i in range(1, 11):
                model.learn(
                    total_timesteps=TIMESTEPS,
                    reset_num_timesteps=False,
                    tb_log_name=algorithm
                )
                node.get_logger().info(f"Model {TIMESTEPS*i} has been trained")
                model.save(f"{models_dir}/{algorithm}/{TIMESTEPS*i}")
                node.get_logger().info(f"Model {TIMESTEPS*i} has been saved")
        except KeyboardInterrupt:
            model.save(f"{models_dir}/{algorithm}/{TIMESTEPS*i}")
    elif node._training_mode == 'retrain':
        model_path = f"{models_dir}/P22_PPO/base.zip"
        model = PPO.load(model_path, env=env)
        node.get_logger().info(f"Base model has been loaded")

        try:
            TIMESTEPS = 10000
            for i in range(1, 21):
                model.learn(
                    total_timesteps=TIMESTEPS,
                    reset_num_timesteps=False,
                    tb_log_name=algorithm
                )
                node.get_logger().info(f"Model base_{TIMESTEPS*i} has been trained")
                model.save(f"{models_dir}/{algorithm}/base_{TIMESTEPS*i}")
                node.get_logger().info(f"Model base_{TIMESTEPS*i} has been saved")
        except KeyboardInterrupt:
            model.save(f"{models_dir}/{algorithm}/base_{TIMESTEPS*i}")
    elif node._training_mode == 'display':
        TIMESTEPS = 10000
        model_path = f"{models_dir}/display/base_200000_p22"
        model = PPO.load(model_path, env=env)
        node.get_logger().info(f"Display model has been loaded")

        try:
            model.learn(
                total_timesteps=TIMESTEPS,
                reset_num_timesteps=False,
                tb_log_name="/home/zixin/Desktop/trash"
            )
        except KeyboardInterrupt:
            pass

    env.close()

    node.get_logger().info("This episode of training has finished.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
