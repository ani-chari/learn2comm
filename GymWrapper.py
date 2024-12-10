import gymnasium as gym
import numpy as np
from gymnasium import spaces
import path_maker
from simulator import Simulator
from infrastructure import Infrastructure
import random

class GymWrapper(gym.Env):

    metadata = {} # TODO

    def __init__(self):
        super(GymWrapper, self).__init__()

        self.m = 5
        self.n = 5
        self.walls = 10

        self.t_max = 100

        self.action_space = spaces.MultiBinary(shape=(self.n, self.n, self.m))

        self.observation_space = spaces.Dict()
        self.observation_space['agent_states'] = spaces.Box(shape=(4, self.n), dtype=np.float32)
        self.observation_space['agent_beliefs'] = spaces.Box(shape=(2, self.n), dtype=np.float32)
        self.observation_space['agent_observations'] = spaces.Box(shape=(2, self.n), dtype=np.float32)
        self.observation_space['subject_states'] = spaces.Box(shape=(4, self.m), dtype=np.float32)
        self.observation_space['wall_locations'] = spaces.Box(shape=(4, self.num_walls))

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

        # self.num_walls = random.randint(10, 20)
        for i in range(self.num_walls):
            p1 = np.array([random.uniform(-100, 100), random.uniform(-100, 100)])
            eps = 50
            p2 = np.array([p1[0] + random.uniform(-eps, eps), p1[1] + random.uniform(-eps, eps)])
            x['walls'].append((p1, p2))

        dt = 0.1
        sim = Simulator(x, self.m, self.n, dt)
        infra = Infrastructure(sim)
        sim.add_infra(infra)

        return sim

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.sim = self.initialize_sim()
        return _, {} # TODO: return correctly formatted initial observation

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

        new_observations = # TODO

        reward = # TODO

        # stop if all agents reached goal or time limit reached
        terminated = -1 not in self.sim.done or self.sim.t == self.t_max
        truncated = False
        info = {}

        return (new_observations, reward, terminated, truncated, info)

    def render(self):
        pass

    def close(self):
        pass