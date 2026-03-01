import rclpy
import gymnasium as gym
from rclpy.node import Node
from gymnasium.envs import registration
from stable_baselines3 import PPO
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
        self.declare_parameter('training_mode', 'train')
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

    # Register custom environment
    # registration.register(
    #     id="Simplified-V0",
    #     entry_point="turtle_rl.simplified_env:simplified_env",
    #     max_episode_steps=150
    # )
    # node.get_logger().info("The simplified_env has been resgistered successfully.")

    # Make custom environment
    # env = gym.make("Simplified-V0")

    env = simplified_env()
    env = Monitor(env)

    # env = DummyVecEnv([lambda: simplified_env()])
    # env = VecCheckNan(env, raise_exception=True)

    # Reset the environment first
    env.reset()

    # Check custom environment to see if it is fine
    # env_checker.check_env(env)
    # node.get_logger().info("The simplified_env has been checked.")

    # Create callbacks needed for training
    # stop_callback = callbacks.StopTrainingOnRewardThreshold(reward_threshold=1500, verbose=1)
    # eval_callback = callbacks.EvalCallback(
    #     eval_env=env,
    #     callback_on_new_best=stop_callback,
    #     eval_freq=100000,
    #     best_model_save_path=models_dir,
    #     n_eval_episodes=50
    # )

    # Training!
    if node._training_mode == 'train':
        try:
            model = PPO(
                policy="MlpPolicy",
                env=env,
                verbose=1,
                tensorboard_log=logs_dir,
                learning_rate=0.0001
            )

            # model.learn(
            #     total_timesteps=int(50000000),
            #     reset_num_timesteps=False,
            #     callback=eval_callback,
            #     tb_log_name=f"{algorithm}"
            # )

            TIMESTEPS = 100000
            model.learn(
                total_timesteps=TIMESTEPS,
                reset_num_timesteps=False,
                tb_log_name=algorithm
            )
            node.get_logger().info(f"Model {TIMESTEPS} has been trained")
            model.save(f"{models_dir}/{algorithm}/{TIMESTEPS}")
            node.get_logger().info(f"Model {TIMESTEPS} has been saved")
        except KeyboardInterrupt:
            model.save(f"{models_dir}/{algorithm}/{TIMESTEPS}")
    elif node._training_mode == 'retrain':
        TIMESTEPS = 100000
        model_path = f"{models_dir}/{algorithm}/{TIMESTEPS*node._epoch}.zip"
        model = PPO.load(model_path, env=env)
        node.get_logger().info(f"Model {TIMESTEPS*(node._epoch)} has been loaded")

        try:
            model.learn(
                total_timesteps=TIMESTEPS,
                reset_num_timesteps=False,
                tb_log_name=algorithm
            )
            node.get_logger().info(f"Model {TIMESTEPS*(node._epoch+1)} has been trained")
            model.save(f"{models_dir}/{algorithm}/{TIMESTEPS*(node._epoch+1)}")
            node.get_logger().info(f"Model {TIMESTEPS*(node._epoch+1)} has been saved")
        except KeyboardInterrupt:
            model.save(f"{models_dir}/{algorithm}/{TIMESTEPS*(node._epoch+1)}")

    env.close()

    node.get_logger().info("This episode of training has finished.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
