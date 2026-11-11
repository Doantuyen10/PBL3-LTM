"""UAV-BS observation/action contract used by the Webots PPO adapter.

Adapted from Downloads/uav_bs_ppo.py on 2026-11-10 (environment only).
Checkpoint training provenance is unconfirmed; this is an explicit deployment
contract, not proof that the checkpoint was trained with this file.
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces


# ---------------------------------------------------------------- mô hình vật lý
def propulsion_power(V):
    """Công suất đẩy UAV cánh quạt (Zeng et al., 2019), đơn vị W."""
    P0, Pi, Utip, v0 = 79.86, 88.63, 120.0, 4.03
    d0, rho, s, A = 0.6, 1.225, 0.05, 0.503
    blade = P0 * (1 + 3 * V**2 / Utip**2)
    induced = Pi * np.sqrt(np.sqrt(1 + V**4 / (4 * v0**4)) - V**2 / (2 * v0**2))
    parasite = 0.5 * d0 * rho * s * A * V**3
    return blade + induced + parasite


def snr_a2g(uav_pos, user_pos, Pt=0.1, noise=1e-12, fc=2e9,
            a=9.61, b=0.16, eta_los=1.0, eta_nlos=20.0):
    """SNR kênh air-to-ground theo xác suất LoS (Al-Hourani, đô thị)."""
    d_vec = uav_pos[None, :] - user_pos                    # (K,3)
    r = np.linalg.norm(d_vec[:, :2], axis=1)
    d = np.linalg.norm(d_vec, axis=1)
    theta = np.degrees(np.arctan2(uav_pos[2], r))
    p_los = 1.0 / (1.0 + a * np.exp(-b * (theta - a)))
    fspl = 20 * np.log10(4 * np.pi * fc * d / 3e8)
    pl_db = fspl + p_los * eta_los + (1 - p_los) * eta_nlos
    return Pt * 10 ** (-pl_db / 10) / noise


# ---------------------------------------------------------------- môi trường
class UAVBSEnv(gym.Env):
    def __init__(self, K=4, area=1000.0, H=100.0, T=200, dt=1.0,
                 v_max=20.0, a_max=5.0, bandwidth=1e6, battery=40000.0,
                 w_rate=1.0, w_fair=0.5, w_energy=0.5, w_bound=0.5):
        super().__init__()
        self.K, self.area, self.H, self.T, self.dt = K, area, H, T, dt
        self.v_max, self.a_max, self.B, self.E_max = v_max, a_max, bandwidth, battery
        self.w_rate, self.w_fair, self.w_energy, self.w_bound = w_rate, w_fair, w_energy, w_bound
        self.P_hover = propulsion_power(0.0)

        # action: gia tốc ngang chuẩn hóa [-1,1]^2
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        # obs: pos(2) vel(2) battery(1) time(1) rel_user(2K) rate(K)
        obs_dim = 6 + 3 * self.K
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)

    # ---- helpers
    def _rates(self):
        snr = snr_a2g(self.pos, self.users)
        spectral = np.log2(1 + snr)                        # bit/s/Hz mỗi user
        return spectral

    def _obs(self):
        rel = (self.users[:, :2] - self.pos[:2]) / self.area
        spec = self._rates()
        return np.concatenate([
            self.pos[:2] / self.area * 2 - 1,
            self.vel / self.v_max,
            [self.E / self.E_max, 1 - self.t / self.T],
            rel.ravel(),
            spec / 10.0,
        ]).astype(np.float32)

    # ---- gym API
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        self.users = np.column_stack([rng.uniform(0.1, 0.9, (self.K, 2)) * self.area,
                                      np.zeros(self.K)])
        self.pos = np.array([rng.uniform(0.2, 0.8) * self.area,
                             rng.uniform(0.2, 0.8) * self.area, self.H])
        self.vel = np.zeros(2)
        self.E, self.t = self.E_max, 0
        self.bits, self.energy_used = 0.0, 0.0
        return self._obs(), {}

    def step(self, action):
        acc = np.clip(action, -1, 1) * self.a_max
        self.vel = self.vel + acc * self.dt
        speed = np.linalg.norm(self.vel)
        if speed > self.v_max:
            self.vel *= self.v_max / speed
        self.pos[:2] += self.vel * self.dt

        # biên: kẹp vị trí, triệt tiêu vận tốc, phạt
        hit = False
        for i in range(2):
            if self.pos[i] < 0 or self.pos[i] > self.area:
                self.pos[i] = np.clip(self.pos[i], 0, self.area)
                self.vel[i] = 0.0
                hit = True

        # truyền thông (băng thông chia đều cho K user)
        spec = self._rates()
        rate_k = (self.B / self.K) * spec                  # bit/s
        step_bits = rate_k.sum() * self.dt
        self.bits += step_bits

        # năng lượng
        V = np.linalg.norm(self.vel)
        P = propulsion_power(V)
        self.E -= P * self.dt
        self.energy_used += P * self.dt

        # reward: các thành phần đã chuẩn hóa ~[0,1]
        r_rate = spec.mean() / 10.0
        r_fair = spec.min() / 10.0                         # công bằng max-min
        r_energy = P / self.P_hover
        reward = (self.w_rate * r_rate + self.w_fair * r_fair
                  - self.w_energy * r_energy - self.w_bound * float(hit))

        self.t += 1
        terminated = bool(self.E <= 0)
        if terminated:
            reward -= 5.0                                  # hết pin
        truncated = self.t >= self.T
        info = {"bits": self.bits, "energy": self.energy_used,
                "EE": self.bits / max(self.energy_used, 1e-9)}   # bit/Joule
        return self._obs(), float(reward), terminated, truncated, info


