import rclpy
import gymnasium as gym
from rclpy.node import Node
from gymnasium.envs import registration
from stable_baselines3 import PPO
from stable_baselines3.common import env_checker
from stable_baselines3.common import callbacks
from stable_baselines3.common.monitor import Monitor

import os

class training_node(Node):
    """training_node class."""

    def __init__(self):
        super().__init__('training_node')
        self.get_logger().info("The training node has just been created.")

        # Parameters

async def main(args=None):
    # Initialize training node
    rclpy.init()
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
    registration.register(
        id="Simplified-V0",
        entry_point="turtle_rl.simplified_env:simplified_env",
        max_episode_steps=150
    )
    node.get_logger().info("The simplified_env has been resgistered successfully.")

    # Make custom environment
    env = gym.make("Simplified-V0")
    env = Monitor(env)
    await env.reset()
    node.get_logger().info("\n Initial reset. \n")

    # Check custom environment to see if it is fine
    # env_checker.check_env(env)
    # node.get_logger().info("The simplified_env has been checked.")

    # Create callbacks needed for training
    stop_callback = callbacks.StopTrainingOnRewardThreshold(reward_threshold=1500, verbose=1)
    eval_callback = callbacks.EvalCallback(
        eval_env=env,
        callback_on_new_best=stop_callback,
        eval_freq=100000,
        best_model_save_path=models_dir,
        n_eval_episodes=50
    )

    # Training!
    model = PPO(
        policy="MultiInputPolicy",
        env=env,
        verbose=1,
        tensorboard_log=logs_dir,
        n_steps=20480,
        gamma=0.99,
        gae_lambda=0.95,
        ent_coef=0.0,
        vf_coef=0.5,
        learning_rate=0.00001,
        clip_range=0.2
    )
    try:
        model.learn(
            total_timesteps=int(50000000),
            reset_num_timesteps=False,
            callback=eval_callback,
            tb_log_name=f"{algorithm}"
        )
    except KeyboardInterrupt:
        model.save(f"{models_dir}/{algorithm}")
    # Save the trained model
    model.save(f"{models_dir}/{algorithm}")

if __name__ == "__main__":
    main()
