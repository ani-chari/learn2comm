import gymnasium as gym
import numpy as np
from gymnasium import spaces
import path_maker
from simulator import Simulator
from infrastructure import Infrastructure
import random

class LearningEnv(gym.Env):

    metadata = {} # TODO

    def __init__(self, m=5, n=5, num_walls=10, to_render=False):
        super(LearningEnv, self).__init__()
        self.to_render = to_render

        self.m = m
        self.n = n
        self.num_walls = num_walls
        self.t_max = 100

        # Flatten to 1D
        self.action_space = spaces.MultiBinary(self.n * self.n * self.m)

        # Flatten each component of the observation space
        self.observation_space = spaces.Dict({
            'agent_states': spaces.Box(low=-np.inf, high=np.inf, 
                                       shape=(self.n * 4,), dtype=np.float64),
            'agent_beliefs': spaces.Box(low=-np.inf, high=np.inf, 
                                        shape=(self.n * 8,), dtype=np.float64),
            'agent_observations': spaces.Box(low=-np.inf, high=np.inf, 
                                             shape=(self.n * 4,), dtype=np.float64),
            'agent_goals': spaces.Box(low=-100, high=100, 
                                      shape=(self.n * 2,), dtype=np.float64),
            'subject_states': spaces.Box(low=-np.inf, high=np.inf, 
                                         shape=(self.m * 4,), dtype=np.float64),
            'wall_locations': spaces.Box(low=-100, high=100, 
                                         shape=(self.num_walls * 4,), dtype=np.float64)
        })


    def initialize_sim(self):

        x = {'pH': [], 'vH': [], 'aH': [], 'gH': [], 'path': [],
            'pR': [], 'vR': [], 'aR': [], 'gR': [],
            'walls': []}
        
        bounds = (-100, 100)

        for i in range(self.m):
            x['pH'].append(np.array([random.uniform(bounds[0], bounds[1]), 
                                     random.uniform(bounds[0], bounds[1])]))
            x['gH'].append(np.array([random.uniform(bounds[0], bounds[1]), 
                                     random.uniform(bounds[0], bounds[1])]))
            x['path'].append(path_maker.make_path(x['pH'][i], x['gH'][i]))
            x['vH'].append(np.array([0., 0.]))
            x['aH'].append(np.array([0., 0.]))

        for i in range(self.n):
            x['pR'].append(np.array([random.uniform(bounds[0], bounds[1]), 
                                     random.uniform(bounds[0], bounds[1])]))
            x['gR'].append(np.array([random.uniform(bounds[0], bounds[1]), 
                                     random.uniform(bounds[0], bounds[1])]))
            x['vR'].append(np.array([0., 0.]))
            x['aR'].append(np.array([0., 0.]))

        eps = 50
        for i in range(self.num_walls):
            p1 = np.array([random.uniform(bounds[0], bounds[1]), 
                        random.uniform(bounds[0], bounds[1])])
            p2 = np.array([
                np.clip(p1[0] + random.uniform(-eps, eps), bounds[0], bounds[1]),
                np.clip(p1[1] + random.uniform(-eps, eps), bounds[0], bounds[1])
            ])
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

        obs['agent_states'] = np.concatenate([
            np.concatenate([self.sim.x['pR'][i], self.sim.x['vR'][i]]) 
            for i in range(self.n)
        ])

        obs['agent_beliefs'] = np.concatenate([
            np.concatenate([
                self.sim.infra.bel[i][0]['pos'][0],
                self.sim.infra.bel[i][0]['vel'][0],
                np.sqrt(self.sim.infra.bel[i][0]['pos'][1]),
                np.sqrt(self.sim.infra.bel[i][0]['vel'][1])
            ]) 
            for i in range(self.n)
        ])

        obs['agent_observations'] = np.concatenate([
            np.concatenate([
                self.sim.infra.obs[i][0]['pos'][-1] if self.sim.infra.obs[i][0]['pos'] else np.zeros(2),
                self.sim.infra.obs[i][0]['vel'][-1] if self.sim.infra.obs[i][0]['vel'] else np.zeros(2)
            ])
            for i in range(self.n)
        ])

        obs['agent_goals'] = np.concatenate([
            self.sim.x['gR'][i] 
            for i in range(self.n)
        ])

        obs['subject_states'] = np.concatenate([
            np.concatenate([self.sim.x['pH'][i], self.sim.x['vH'][i]]) 
            for i in range(self.m)
        ])

        obs['wall_locations'] = np.concatenate([
            np.concatenate([w[0], w[1]]) 
            for w in self.walls
        ])

        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.sim = self.initialize_sim()

        self.goal_dists = self.compute_goal_dists()

        return self.formatted_observation(), {}

    def step(self, action):

        # Reshape flattened action back to 3D
        action_3d = action.reshape((self.n, self.n, self.m))

        # execute communications
        for a in range(self.n):
            for b in range(self.n):
                for h in range(self.m):
                    if action_3d[a][b][h]:
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
        terminated = -1 not in self.sim.done
        truncated = self.sim.t == self.t_max
        info = {}

        return (new_observation, reward, terminated, truncated, info)

    def render(self):
        if self.to_render:
            self.sim.vis()

    def close(self):
        pass