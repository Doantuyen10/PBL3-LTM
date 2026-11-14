"""Closed-loop UDP client for the ns-3.47 co-simulation server (epuck-ns3-server)."""
from pathlib import Path
import json
import socket
import subprocess
import time
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[4]
NS3_BINARY = PROJECT_ROOT / 'tools/ns-3.47/build/scratch/ns3.47-epuck-ns3-server-default'
DEFAULT_NS3_PORT = 8082


def _find_free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def _is_udp_port_free(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind(('127.0.0.1', int(port)))
            return True
        except OSError:
            return False


class Ns3CoSimClient:
    """Synchronous low-latency UDP bridge to ns-3.47 Mobility & PropagationLossModel."""

    def __init__(self, port=DEFAULT_NS3_PORT, auto_spawn=True, quiet_spawn=True, timeout=0.25):
        self.port = _find_free_udp_port() if port == 0 else int(port)
        self.addr = ('127.0.0.1', self.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.proc = None
        self.last_feedback = None

        if not self._probe():
            if not auto_spawn:
                self.sock.close()
                raise RuntimeError(f'Không kết nối được ns-3 tại 127.0.0.1:{self.port}')
            if not NS3_BINARY.is_file():
                self.sock.close()
                raise FileNotFoundError(f'Thiếu binary ns-3: {NS3_BINARY}')
            if not _is_udp_port_free(self.port):
                self.port = _find_free_udp_port()
                self.addr = ('127.0.0.1', self.port)
            cmd = [
                str(NS3_BINARY),
                f'--port={self.port}',
                f'--quiet={"true" if quiet_spawn else "false"}',
            ]
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL if quiet_spawn else None,
                stderr=subprocess.DEVNULL if quiet_spawn else None,
            )
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                if self._probe():
                    break
                time.sleep(0.02)
            else:
                self.close()
                raise RuntimeError(f'Khởi chạy ns-3 server tại cổng {self.port} thất bại.')

    def _probe(self):
        try:
            payload = json.dumps({
                'type': 'STATE_QUERY',
                'uav_pos': [0.0, 0.0, 30.0],
                'users_pos': [-100.0, -50.0, 0.0, 50.0, -50.0, 0.0, 100.0, 100.0, 0.0, 2.0, 0.0, 0.0],
                'sim_time': 0.0,
            }).encode('utf-8')
            self.sock.sendto(payload, self.addr)
            data, _ = self.sock.recvfrom(4096)
            resp = json.loads(data.decode('utf-8'))
            if isinstance(resp, dict) and resp.get('type') == 'NS3_FEEDBACK':
                self.last_feedback = resp
                return True
        except (OSError, ValueError):
            return False
        return False

    def _request(self, payload_dict):
        raw = json.dumps(payload_dict).encode('utf-8')
        self.sock.sendto(raw, self.addr)
        data, _ = self.sock.recvfrom(4096)
        resp = json.loads(data.decode('utf-8'))
        if 'spectral_eff' in resp:
            self.last_feedback = resp
        return resp

    def reset_stats(self, shadow_loss_db=None):
        msg = {'type': 'RESET_STATS'}
        if shadow_loss_db is not None:
            msg['shadow_loss_db'] = [float(x) for x in shadow_loss_db]
        return self._request(msg)

    def configure_shadowing(self, shadow_loss_db):
        vals = [float(x) for x in np.asarray(shadow_loss_db, dtype=float).ravel()]
        if len(vals) != 4:
            raise ValueError('shadow_loss_db phải gồm đúng 4 giá trị (dB) cho 3 client và server.')
        return self._request({'type': 'CONFIG_SHADOWING', 'shadow_loss_db': vals})

    def query_state(self, uav_pos, users_pos, sim_time=0.0):
        uav = np.asarray(uav_pos, dtype=float).ravel().tolist()
        users = np.asarray(users_pos, dtype=float).ravel().tolist()
        return self._request({
            'type': 'STATE_QUERY',
            'uav_pos': uav,
            'users_pos': users,
            'sim_time': float(sim_time),
        })

    def evaluate_packet(self, packet, uav_pos, users_pos, sim_time=0.0):
        uav = np.asarray(uav_pos, dtype=float).ravel().tolist()
        users = np.asarray(users_pos, dtype=float).ravel().tolist()
        req = dict(packet)
        req.update({
            'type': 'DATA_PACKET',
            'uav_pos': uav,
            'users_pos': users,
            'sim_time': float(sim_time),
        })
        return self._request(req)

    def close(self):
        if self.sock is not None:
            if self.proc is not None and self.proc.poll() is None:
                try:
                    self.sock.sendto(b'{"type":"SHUTDOWN"}', self.addr)
                except OSError:
                    pass
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        if self.proc is not None:
            try:
                self.proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None
