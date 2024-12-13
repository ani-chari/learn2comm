from stable_baselines3.common.env_checker import check_env
from learning_env import LearningEnv

env = LearningEnv()
check_env(env)