"""Three-client relay task, shared unchanged between training and Webots.

Observation order follows UAVBSEnv (18 entries): pos, velocity, battery, time,
relative xy of client 1/2/3/server, spectral efficiencies of these four nodes.
The server is NOT a fourth e-puck client; all four links enter the constraint.
"""
import numpy as np
from uav_bs_env import UAVBSEnv

CONTRACT = 'relay_3epuck_v1'
BASE_USERS = np.array([[400.,450.,0.],[550.,450.,0.],[600.,600.,0.],[502.,500.,0.]])

class RelayEnv(UAVBSEnv):
    def __init__(self, randomize=True, H=30.0):
        super().__init__(K=4,H=H,T=200,dt=1.,v_max=5.,a_max=2.)
        self.randomize = randomize
        self.comm_range = 150.0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.users = BASE_USERS.copy()
        self.pos = np.array([500.,500.,self.H])
        if self.randomize:
            # All train topologies remain close to the actual, feasible world.
            self.users[:,:2] += self.np_random.uniform(-5,5,(4,2))
            extent = 25. if self.np_random.random() < .85 else 70.
            self.pos[:2] += self.np_random.uniform(-extent,extent,2)
        if options:
            if 'users' in options:self.users = np.asarray(options['users'],float).copy()
            if 'position' in options:self.pos = np.asarray(options['position'],float).copy()
        return self._obs(), {}

    def step(self, action):
        obs, _, terminated, truncated, info = super().step(action)
        distances = np.linalg.norm(self.users-self.pos,axis=1)
        connected = distances <= self.comm_range
        excess = np.maximum(distances-self.comm_range,0)/self.comm_range
        worst = distances.max()/self.comm_range
        speed = np.linalg.norm(self.vel)/self.v_max
        # Dense worst-link objective, explicit outage penalty and stopping cost.
        # No coverage clipping/shield/fallback: all movement is the PPO action.
        reward = (3.0*float(connected.all()) + connected.mean()
                  - 3.0*worst - 12.0*excess.sum()
                  - .3*speed**2 - .03*float(np.square(action).mean()))
        if terminated:reward-=5
        info.update(all_clients=bool(connected[:3].all()),all_links=bool(connected.all()),
                    max_client_distance=float(distances[:3].max()),
                    max_link_distance=float(distances.max()),distances=distances.tolist())
        return obs,float(reward),terminated,truncated,info
