import gymnasium as gym
import numpy as np
from gymnasium import spaces
import path_maker
from simulator import Simulator
from infrastructure import Infrastructure
import random

class GymWrapper(gym.Env):

    metadata = {} # TODO

    def __init__(self, to_render=False):
        super(GymWrapper, self).__init__()
        self.to_render = to_render

        self.m = 5
        self.n = 5
        self.walls = 10

        self.t_max = 100

        self.action_space = spaces.MultiBinary(shape=(self.n, self.n, self.m))

        self.observation_space = spaces.Dict()
        self.observation_space['agent_states'] = spaces.Box(shape=(self.n, 4), dtype=np.float32)
        self.observation_space['agent_beliefs'] = spaces.Box(shape=(self.n, 4), dtype=np.float32)
        self.observation_space['agent_observations'] = spaces.Box(shape=(self.n, 4), dtype=np.float32)
        self.observation_space['agent_goals'] = spaces.Box(shape=(self.n, 2), dtype=np.float32)
        self.observation_space['subject_states'] = spaces.Box(shape=(self.m, 4), dtype=np.float32)
        self.observation_space['wall_locations'] = spaces.Box(shape=(self.num_walls, 4))

    def initialize_sim(self):

        x = {'pH': [], 'vH': [], 'aH': [], 'gH': [], 'path': [],
            'pR': [], 'vR': [], 'aR': [], 'gR': [],
            'walls': []}

        for i in range(self.m):
            x['pH'].append(np.array([random.uniform(-100, 100), random.uniform(-100, 100)]))
            x['gH'].append(np.array([random.uniform(-100, 100), random.uniform(-100, 100)]))
            x['path'].append(path_maker.make_path(x['pH'][i], x['gH'][i]))
            x['vH'].append(np.array([0., 0.]))
            x['aH'].append(np.array([0., 0.]))

        for i in range(self.n):
            x['pR'].append(np.array([random.uniform(-100, 100), random.uniform(-100, 100)]))
            x['gR'].append(np.array([random.uniform(-100, 100), random.uniform(-100, 100)]))
            x['vR'].append(np.array([0., 0.]))
            x['aR'].append(np.array([0., 0.]))

        for i in range(self.num_walls):
            p1 = np.array([random.uniform(-100, 100), random.uniform(-100, 100)])
            eps = 50
            p2 = np.array([p1[0] + random.uniform(-eps, eps), p1[1] + random.uniform(-eps, eps)])
            x['walls'].append((p1, p2))
        self.walls = x['walls']

        dt = 0.1
        sim = Simulator(x, self.m, self.n, dt)
        infra = Infrastructure(sim)
        sim.add_infra(infra)

        return sim
    
    def compute_goal_dists(self):
        normsq = lambda x : x.T @ x
        return sum([normsq(self.sim.x['pR'][i] - self.sim.x['gR'][i]) for i in range(self.n)])
    
    def formatted_observation(self):
        obs = {}
        obs['agent_states'] = np.stack([np.concatenate([self.sim.x['pR'][i], self.sim.x['vR'][i]]) for i in range(self.n)])
        obs['agent_beliefs'] = np.stack([np.asarray(self.sim.infra.bel[i]).flatten() for i in range(self.n)])
        obs['agent_observations'] = np.stack([np.asarray(self.sim.infra.obs[i]).flatten() for i in range(self.n)])
        obs['agent_goals'] = np.stack([self.sim.x['gR'][i] for i in range(self.n)])
        obs['subject_states'] = np.stack([np.concatenate([self.sim.x['pH'][i], self.sim.x['vH'][i]]) for i in range(self.m)])
        obs['wall_locations'] = np.stack([np.asarray(self.walls[i]).flatten() for i in range(self.num_walls)])
        return obs


    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.sim = self.initialize_sim()

        self.goal_dists = self.compute_goal_dists()

        return self.formatted_observation(), {}

    def step(self, action):

        # execute communications
        for a in range(self.n):
            for b in range(self.n):
                for h in range(self.m):
                    if action[a][b][h]:
                        self.sim.infra.share(a, b, h)

        # iterate simulation
        x = self.sim.move()

        # run individual sensing
        for r in range(self.n):
            for h in range(self.m):
                self.sim.infra.sense(r, h)

        # get new observation
        new_observation = self.formatted_observation()

        # compute reward as weighted sum of progress and bandwidth use
        new_goal_dists = self.compute_goal_dists()
        progress = new_goal_dists - self.goal_dists
        self.goal_dists = new_goal_dists
        bandwidth_cost = np.sum(action) # TODO: include unique bandwidth costs per comm?
        alpha = 1 # TODO: tune
        reward = progress - alpha * bandwidth_cost # TODO: include fixed end reward for agent reaching goal?

        # stop if all agents reached goal or time limit reached
        terminated = -1 not in self.sim.done or self.sim.t == self.t_max
        truncated = False
        info = {}

        return (new_observation, reward, terminated, truncated, info)

    def render(self):
        if self.to_render:
            self.sim.vis()

    def close(self):
        pass