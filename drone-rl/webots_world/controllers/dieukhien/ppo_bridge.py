"""PPO inference and closed-loop ns-3.47 channel/control adapter."""
from pathlib import Path
import json
import hashlib
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from relay_env import RelayEnv, CONTRACT
from ns3_cosim import Ns3CoSimClient, DEFAULT_NS3_PORT

# Stable order: three sources, then the ground server as the fourth user.
USER_DEFS = ('E_PUCK_CLIENT_1', 'E_PUCK_CLIENT_2', 'E_PUCK_CLIENT_3', 'E_PUCK_SERVER')
WORLD_OFFSET = np.array([500.0, 500.0, 0.0])
POLICY_DT = 1.0
TARGET_ALTITUDE = 30.0


class PPOBridge:
    def __init__(self, folder, altitude=TARGET_ALTITUDE, use_ns3=True,
                 ns3_port=DEFAULT_NS3_PORT, ns3_control_assist=True, ns3_client=None):
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
        self.use_ns3 = bool(use_ns3)
        self.ns3_control_assist = bool(ns3_control_assist)
        self._owns_ns3 = False
        self.ns3 = None
        self.last_ns3_feedback = None
        try:
            self.model = PPO.load(folder / 'ppo_relay_3epuck.zip', env=self.vec, device='cpu')
            if self.model.observation_space.shape != (18,) or self.model.action_space.shape != (2,):
                raise ValueError('Checkpoint phải có observation (18,) và action (2,).')
            self.env = RelayEnv(randomize=False, H=self.altitude)
            self.env.reset(seed=0)
            self._base_rates = self.env._rates
            self.env._rates = self._rates_with_ns3
            if self.use_ns3:
                if ns3_client is not None:
                    self.ns3 = ns3_client
                else:
                    self.ns3 = Ns3CoSimClient(port=ns3_port, auto_spawn=True, quiet_spawn=True)
                    self._owns_ns3 = True
            self.started = False
            self.done = False
        except Exception:
            if self._owns_ns3 and self.ns3 is not None:
                self.ns3.close()
            self.vec.close()
            raise

    def _rates_with_ns3(self):
        if self.use_ns3 and self.last_ns3_feedback is not None:
            spec = np.asarray(self.last_ns3_feedback['spectral_eff'], dtype=np.float64)
            if spec.shape == (4,) and np.isfinite(spec).all():
                return spec
        return self._base_rates()

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
        if self.use_ns3 and self.ns3 is not None:
            ground_users = users.copy()
            ground_users[:, 2] = 0.0
            self.last_ns3_feedback = self.ns3.query_state(
                pos, ground_users, sim_time=float(self.env.t)
            )

    def start(self, position, users, shadow_loss_db=None):
        self.env.reset(seed=0)
        if self.use_ns3 and self.ns3 is not None:
            self.ns3.reset_stats(shadow_loss_db=shadow_loss_db)
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
        ppo_action = np.asarray(self.model.predict(normalized, deterministic=True)[0][0], dtype=np.float64)
        if ppo_action.shape != (2,) or not np.isfinite(ppo_action).all() or np.any(np.abs(ppo_action) > 1):
            raise ValueError('Hành động PPO không hợp lệ.')

        # Closed-loop ns-3 participation: combine PPO action with PD-damped ns-3 restorative
        # acceleration whenever ns-3 detects degraded SNR, obstacle shadowing, or boundary proximity.
        action = ppo_action.copy()
        assist_weight = 0.0
        ns3_accel = np.zeros(2, dtype=np.float64)
        if self.use_ns3 and self.ns3_control_assist and self.last_ns3_feedback is not None:
            assist_weight = float(np.clip(self.last_ns3_feedback.get('assist_weight', 0.0), 0.0, 1.0))
            pull_dir = np.asarray(self.last_ns3_feedback.get('control_accel', [0.0, 0.0]), dtype=np.float64)
            if assist_weight > 0.0 and pull_dir.shape == (2,) and np.isfinite(pull_dir).all():
                # Also prevent pulling any other client past 143 m
                dists = np.asarray(self.last_ns3_feedback.get('distances_m', [0, 0, 0, 0]), dtype=np.float64)
                guard = np.zeros(2, dtype=np.float64)
                for idx in range(4):
                    if dists[idx] > 142.0:
                        rel_xy = self.env.users[idx, :2] - self.env.pos[:2]
                        norm_xy = float(np.linalg.norm(rel_xy))
                        if norm_xy > 1e-6:
                            guard += ((dists[idx] - 142.0) / 8.0) * (rel_xy / norm_xy)
                combined_dir = pull_dir + guard
                dir_norm = float(np.linalg.norm(combined_dir))
                if dir_norm > 1e-6:
                    combined_dir = combined_dir / dir_norm
                v_des = self.env.v_max * min(0.8, assist_weight) * combined_dir
                ns3_accel = np.clip((v_des - self.env.vel) / (self.env.a_max * self.env.dt), -1.0, 1.0)
                action = np.clip((1.0 - assist_weight) * ppo_action + assist_weight * ns3_accel, -1.0, 1.0)

        # Action is normalized acceleration. Updates velocity, boundary constraints,
        # battery and episode time at the reference 1 s step.
        _, _, terminated, truncated, info = self.env.step(action)
        self.done = bool(terminated or truncated)
        target = self.env.pos.copy() - WORLD_OFFSET

        if self.last_ns3_feedback is not None:
            info.update(
                ns3_active=True,
                ppo_raw_action=ppo_action.tolist(),
                ns3_assist_weight=assist_weight,
                ns3_control_accel=ns3_accel.tolist(),
                ns3_snr_db=list(self.last_ns3_feedback.get('snr_db', [])),
                ns3_spectral_eff=list(self.last_ns3_feedback.get('spectral_eff', [])),
                ns3_link_ok=list(self.last_ns3_feedback.get('link_ok', [])),
                ns3_weakest_node=self.last_ns3_feedback.get('weakest_node', ''),
                ns3_pdr=float(self.last_ns3_feedback.get('pdr', 1.0)),
                ns3_avg_delay_ms=float(self.last_ns3_feedback.get('avg_delay_ms', 0.0)),
                ns3_throughput_bps=float(self.last_ns3_feedback.get('throughput_bps', 0.0)),
            )
        else:
            info.update(ns3_active=False, ppo_raw_action=ppo_action.tolist(), ns3_assist_weight=0.0)

        return target, action.copy(), info

    def close(self):
        if self._owns_ns3 and self.ns3 is not None:
            self.ns3.close()
            self.ns3 = None
        self.vec.close()
