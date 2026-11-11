"""PPO inference and model-space state; independent of Webots for testing."""
from pathlib import Path
import json
import hashlib
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from relay_env import RelayEnv, CONTRACT

# Stable order: three sources, then the ground server as the fourth user.
USER_DEFS = ('E_PUCK_CLIENT_1', 'E_PUCK_CLIENT_2', 'E_PUCK_CLIENT_3', 'E_PUCK_SERVER')
WORLD_OFFSET = np.array([500.0, 500.0, 0.0])
POLICY_DT = 1.0
TARGET_ALTITUDE = 30.0


class PPOBridge:
    def __init__(self, folder, altitude=TARGET_ALTITUDE):
        self.altitude = float(altitude)
        if not np.isfinite(self.altitude) or self.altitude <= 0:
            raise ValueError("Độ cao PPO phải hữu hạn và lớn hơn 0.")
        folder = Path(folder)
        for name in ('ppo_relay_3epuck.zip', 'vecnorm_relay_3epuck.pkl', 'relay_model.json'):
            if not (folder / name).is_file():
                raise FileNotFoundError(f'Thiếu tệp PPO: {folder / name}')
        metadata = json.loads((folder / 'relay_model.json').read_text())
        if metadata['contract'] != CONTRACT or self.altitude != metadata['altitude_m']:
            raise ValueError('Checkpoint relay yêu cầu đúng contract và độ cao 30 m.')
        checks = {'ppo_relay_3epuck.zip': 'model_sha256',
                  'vecnorm_relay_3epuck.pkl': 'vecnorm_sha256',
                  'relay_env.py': 'env_sha256', 'uav_bs_env.py': 'base_env_sha256'}
        for name, key in checks.items():
            if hashlib.sha256((folder / name).read_bytes()).hexdigest() != metadata[key]:
                raise ValueError(f'Tệp {name} không khớp bộ checkpoint đã đánh giá.')
        self.vec = VecNormalize.load(str(folder / 'vecnorm_relay_3epuck.pkl'),
                                    DummyVecEnv([lambda: RelayEnv(randomize=False, H=self.altitude)]))
        self.vec.training = False
        self.vec.norm_reward = False
        try:
            self.model = PPO.load(folder / 'ppo_relay_3epuck.zip', env=self.vec, device='cpu')
            if self.model.observation_space.shape != (18,) or self.model.action_space.shape != (2,):
                raise ValueError('Checkpoint phải có observation (18,) và action (2,).')
            self.env = RelayEnv(randomize=False, H=self.altitude)
            self.env.reset(seed=0)
            self.started = False
            self.done = False
        except Exception:
            self.vec.close()
            raise

    @staticmethod
    def _position(position):
        pos = np.asarray(position, dtype=np.float64)
        if pos.shape != (3,) or not np.isfinite(pos).all():
            raise ValueError('Vị trí UAV phải là vector 3 chiều hữu hạn.')
        if np.any(np.abs(pos[:2]) > 500):
            raise ValueError('UAV nằm ngoài vùng Webots [-500, 500] m.')
        return pos

    def _sync(self, position, users):
        pos = self._position(position)
        users = np.asarray(users, dtype=np.float64)
        if users.shape != (4, 3) or not np.isfinite(users).all():
            raise ValueError('Cần đúng 4 vị trí node hữu hạn (3 client + server).')
        self.env.pos = pos.copy() + WORLD_OFFSET
        self.env.users = users.copy() + WORLD_OFFSET
        # Ground-plane users; tiny negative Webots contact offsets are ignored.
        self.env.users[:, 2] = 0.0

    def start(self, position, users):
        self.env.reset(seed=0)
        self._sync(position, users)
        if not np.isclose(self.env.pos[2], self.altitude, atol=1e-5):
            raise ValueError(f'PPO bắt đầu ở độ cao {self.altitude:g} m.')
        self.started, self.done = True, False

    def observation(self, position, users):
        if not self.started:
            raise RuntimeError('Phải gọi start sau khi cất cánh.')
        self._sync(position, users)
        obs = self.env._obs()
        if obs.shape != (18,) or not np.isfinite(obs).all():
            raise ValueError('Quan sát PPO không hợp lệ.')
        return obs

    def next_target(self, position, users):
        if self.done:
            raise RuntimeError('Episode đã kết thúc; reset world để chạy lại.')
        obs = self.observation(position, users)
        normalized = self.vec.normalize_obs(obs[None, :])
        if not np.isfinite(normalized).all():
            raise ValueError('Quan sát sau chuẩn hóa không hữu hạn.')
        action = np.asarray(self.model.predict(normalized, deterministic=True)[0][0])
        if action.shape != (2,) or not np.isfinite(action).all() or np.any(np.abs(action) > 1):
            raise ValueError('Hành động PPO không hợp lệ.')
        # Action is normalized acceleration, not velocity. This updates velocity,
        # boundary constraints, battery and episode time at the reference 1 s step.
        _, _, terminated, truncated, info = self.env.step(action)
        self.done = bool(terminated or truncated)
        target = self.env.pos.copy() - WORLD_OFFSET
        return target, action.copy(), info

    def close(self):
        self.vec.close()
