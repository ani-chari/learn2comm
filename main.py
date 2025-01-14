from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from learning_env import LearningEnv

# env = LearningEnv()
# check_env(env)

vec_env = make_vec_env(LearningEnv, n_envs=1, env_kwargs=dict(m=5, n=5, num_walls=10, to_render=True))

# model = PPO("MlpPolicy", vec_env, verbose=1).learn(5000) # TODO: decide whether to format observation space as dict or flatten
model = PPO("MultiInputPolicy", vec_env, verbose=1)
model.learn(total_timesteps=25000)
model.save("comm-v1")

obs = vec_env.reset()
# n_steps = 1000
# for step in range(n_steps):
while True:
    action, _ = model.predict(obs, deterministic=True)
    # action, _ = model.predict(obs) # TODO: decide whether deterministic
    # print(f"Step {step + 1}")
    print("Action: ", action)
    obs, reward, done, info = vec_env.step(action)
    print("obs=", obs, "reward=", reward, "done=", done)
    vec_env.render()
    if done:
        print("Goal reached!", "reward=", reward)
        break
