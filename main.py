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
                             log_path='./logs/', eval_freq=500,
                             deterministic=True, render=False)
tensorboard_callback = TensorboardCallback()

# Train the model
model.learn(total_timesteps=25000, callback=[eval_callback, tensorboard_callback])

# Save the final model
model.save("comm-v1")


# # model = PPO("MlpPolicy", vec_env, verbose=1).learn(5000) # TODO: decide whether to format observation space as dict or flatten
# model = PPO("MultiInputPolicy", env, verbose=1)
# # model.learn(total_timesteps=25000)
# # model.save("comm-v1")

# model.load("comm-v1")

# obs, _ = env.reset()
# # n_steps = 1000
# # for step in range(n_steps):
# while True:
#     action, _ = model.predict(obs, deterministic=True)
#     # action, _ = model.predict(obs) # TODO: decide whether deterministic
#     # print(f"Step {step + 1}")
#     print("Action: ", action)
#     obs, reward, done, stopped, info = env.step(action)
#     print("obs=", obs, "reward=", reward, "done=", done)
#     env.render()

#     # # Add a short pause to update the rendering window
#     # import time
#     # time.sleep(0.01)

#     if done:
#         print("Goal reached!", "reward=", reward)
#         break

# # # Keep the plot open after the loop ends
# # import matplotlib.pyplot as plt
# # plt.show()

# # Things to track/plot during training:
# # policy loss, value loss, cumulative reward -> use tensorboard

# # another idea: have one RL agent running for every pair of robots (decentralized RL), 
# # which enables flexibility in m and n (don't have to fix them and train one policy for each)