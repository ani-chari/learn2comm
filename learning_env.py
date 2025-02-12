import gymnasium as gym
import numpy as np
from gymnasium import spaces
import path_maker
from simulator import Simulator
from infrastructure import Infrastructure
import random

class LearningEnv(gym.Env):

    # TODO: ADD COMM FREQUENCY

    metadata = {} # TODO

    def __init__(self, m=5, n=5, num_walls=10, to_render=False):
        super(LearningEnv, self).__init__()
        self.to_render = to_render

        self.m = m
        self.n = n
        self.num_walls = num_walls

        # Flatten to 1D
        self.action_space = spaces.MultiBinary(self.n * self.n * self.m)

        # Normalize observation space
        self.observation_space = spaces.Dict({
            'agent_states': spaces.Box(low=-1, high=1, shape=(self.n * 4,), dtype=np.float64),
            'agent_beliefs': spaces.Box(low=-1, high=1, shape=(self.n * 8,), dtype=np.float64),
            'agent_observations': spaces.Box(low=-1, high=1, shape=(self.n * 4,), dtype=np.float64),
            'agent_goals': spaces.Box(low=-1, high=1, shape=(self.n * 2,), dtype=np.float64),
            'subject_states': spaces.Box(low=-1, high=1, shape=(self.m * 4,), dtype=np.float64),
            'wall_locations': spaces.Box(low=-1, high=1, shape=(self.num_walls * 4,), dtype=np.float64)
        })

        # Initialize normalization parameters
        self.obs_low = -100
        self.obs_high = 100

        self.max_steps = 10000
        self.current_step = 0

    def normalize_obs(self, obs):
            return 2 * (obs - self.obs_low) / (self.obs_high - self.obs_low) - 1

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

        # Normalize each observation component
        for key in obs:
            obs[key] = self.normalize_obs(obs[key])
        
        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.sim = self.initialize_sim()
        self.goal_dists = self.compute_goal_dists()
        self.current_step = 0
        return self.formatted_observation(), {}

    def step(self, action):

        self.current_step += 1

        # Reshape flattened action back to 3D
        action_3d = action.reshape((self.n, self.n, self.m))

        count = 0
        # execute communications
        for a in range(self.n):
            for b in range(self.n):
                for h in range(self.m):
                    if action_3d[a][b][h]:
                        count += 1
                        self.sim.infra.share(a, b, h)

        # print("TOTAL COMM COUNT:", count)

        # iterate simulation
        x = self.sim.move()

        # run individual sensing
        for r in range(self.n):
            for h in range(self.m):
                self.sim.infra.sense(r, h)

        # get new observation
        new_observation = self.formatted_observation()


        # TODO: NEED TO MAKE TERMINATION CONDITION REINITIALIZE A NEW RANDOMIZED ENVIRONMENT

        # Check if episode should terminate
        terminated = all(self.sim.done[i] != -1 for i in range(self.n))  # All robots reached goals
        truncated = self.current_step >= self.max_steps  # Max steps reached
        info = {}

        if terminated:
            print("TERMINATED", self.current_step)
        if truncated:
            print("TRUNCATED", self.current_step)

        # Shaped reward
        new_goal_dists = self.compute_goal_dists()
        progress = self.goal_dists - new_goal_dists # make progress positive, since we want to maximize
        # old goal dist > new goal dist means progress!
        # print("PROGRESS:", progress)
        self.goal_dists = new_goal_dists
        bandwidth_cost = np.sum(action)
        
        # Shaped reward components
        distance_reward = progress # TODO: maybe instead just simply minimize distance?
        communication_penalty = -0.1 * bandwidth_cost
        goal_reward = 10 if terminated else 0 # TODO: maybe make all rewards negative to be consistent
        time_penalty = -0.01  # Small penalty for each step to encourage faster completion
        
        alpha = [1, 1, 1, 1] # TODO: tune
        # Combine reward components
        # reward = alpha[0] * distance_reward \
        #         + alpha[1] * communication_penalty \
        #         + alpha[2] * goal_reward \
        #         + alpha[3] * time_penalty
        reward = distance_reward + goal_reward + time_penalty + communication_penalty

        # I THINK DISTANCE REWARD IS NEGATIVE WHICH FUCKS IT UP

        return (new_observation, reward, terminated, truncated, info)

    def render(self):
        if self.to_render:
            self.sim.vis()

    def close(self):
        pass