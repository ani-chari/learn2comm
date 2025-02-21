from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from learning_env import LearningEnv
import os

# env = LearningEnv()
# check_env(env)

env = LearningEnv(m=5, n=5, num_walls=10, to_render=True)
# vec_env = make_vec_env(LearningEnv, n_envs=1, env_kwargs=dict(m=5, n=5, num_walls=10, to_render=True))


from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
import tensorflow as tf

class EpisodeCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(EpisodeCallback, self).__init__(verbose)
        self.episode_count = 0

    def _on_step(self):
        if self.training_env.get_attr('current_step')[0] == 0:  # Episode just reset
            self.episode_count += 1
            print(f"Episode {self.episode_count} completed. Total steps: {self.num_timesteps}")
        return True

class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)
        self.cumulative_reward = 0

    def _on_step(self) -> bool:
        self.cumulative_reward += self.locals['rewards'][0]
        self.logger.record('cumulative_reward', self.cumulative_reward)
        return True

# Create log dir
log_dir = "tensorboard_logs"
os.makedirs(log_dir, exist_ok=True)

# Wrap the environment
env = Monitor(env)
env = DummyVecEnv([lambda: env])

# Create the model with tensorboard logging
model = PPO("MultiInputPolicy", env, verbose=1, tensorboard_log=log_dir)

# Set up the callbacks
eval_callback = EvalCallback(env, best_model_save_path='./best_model',
                             log_path='./logs/', eval_freq=5000,
                             deterministic=True, render=False)
tensorboard_callback = TensorboardCallback()
episode_callback = EpisodeCallback()

# Train the model
model.learn(total_timesteps=10000000, callback=[eval_callback, tensorboard_callback, episode_callback]) # TODO: increase timesteps (10x)?

# Save the final model
model.save("comm-v1")

# model.load("comm-v1")

# # Evaluate the model
# obs = env.reset()
# while True:
#     action, _ = model.predict(obs)
#     obs, reward, done, info = env.step(action)
#     env.render()
#     if done:
#         obs = env.reset()

# # Keep the plot open after the loop ends
# import matplotlib.pyplot as plt
# plt.show()

# # Things to track/plot during training:
# # policy loss, value loss, cumulative reward -> use tensorboard

# # another idea: have one RL agent running for every pair of robots (decentralized RL), 
# # which enables flexibility in m and n (don't have to fix them and train one policy for each)