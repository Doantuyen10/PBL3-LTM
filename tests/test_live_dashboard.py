import contextlib
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'drone-rl/webots_world/controllers/dieukhien'))
from live_dashboard import LiveDashboard
from test_ppo_controller import Robot, Socket, FOLDER
from ppo_bridge import PPOBridge
import dieukhien
import numpy as np


def sample():
    return dict(sim_time=1., position=[1., 2., 30.], speed=0., battery=40000.)


class LiveDashboardTests(unittest.TestCase):
    def test_snapshot_is_detached_and_json_safe(self):
        live = LiveDashboard()
        self.assertIsNone(live.snapshot()['state'])
        state = sample()
        state['speed'] = float('nan')
        live.publish(state)
        state['position'][0] = 900
        snap = live.snapshot()
        self.assertEqual(snap['state']['position'][0], 1.)
        self.assertIsNone(snap['state']['speed'])
        json.dumps(snap, allow_nan=False)
        snap['state']['position'][0] = 800
        self.assertEqual(live.snapshot()['state']['position'][0], 1.)

    def test_throttle_and_stale_age(self):
        live = LiveDashboard()
        with patch('live_dashboard.time.monotonic', return_value=10.):
            live.publish(sample())
        with patch('live_dashboard.time.monotonic', return_value=10.05):
            self.assertFalse(live.ready())
            live.publish(dict(sample(), sim_time=2.))
        with patch('live_dashboard.time.monotonic', return_value=14.):
            snap = live.snapshot()
            self.assertEqual(snap['state']['sim_time'], 1.)
            self.assertEqual(snap['age_s'], 4.)
            self.assertTrue(live.ready())

    def test_relay_counts_each_packet_once_and_keeps_ns3_verdict(self):
        live = LiveDashboard()
        class Ns3:
            def evaluate_packet(self, packet, *args, **kwargs):
                return dict(forward_allowed=packet['seq'] % 2 == 0, drop_reason='LOW_SNR',
                            pdr=.5, e2e_delay_ms=5., throughput_bps=1234.)
        packets = [json.dumps(dict(sender='E_PUCK_CLIENT_1', seq=i)).encode() for i in range(40)]
        sock = Socket(packets)
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(2):
                dieukhien.relay_packets(sock, np.array([0., 0., 30.]), Robot().nodes,
                                        ns3_client=Ns3(), live=live)
        live.publish(sample())
        state = live.snapshot()['state']
        self.assertEqual(state['packets']['received'], 40)
        self.assertEqual(state['packets']['forwarded'], 20)
        self.assertEqual(state['packets']['dropped'], 20)
        self.assertEqual(len(sock.sent), 20)
        self.assertEqual(len(state['events']), 30)
        self.assertEqual(state['events'][0]['seq'], 39)
        self.assertEqual(state['packets']['verdict']['pdr'], .5)

    def test_controller_publishes_real_gps_without_supervisor(self):
        bridge = PPOBridge(FOLDER, use_ns3=False)
        self.addCleanup(bridge.close)
        robot = Robot(251)
        live = LiveDashboard()
        with patch.object(live, 'ready', return_value=True), patch('live_dashboard.time.monotonic', side_effect=range(10000)), contextlib.redirect_stdout(io.StringIO()):
            dieukhien.run(robot, bridge, Socket(), live=live)
        state = live.snapshot()['state']
        self.assertEqual(state['position'], robot.devices['gps'].values)
        self.assertEqual(state['policy_step'], 2)
        self.assertEqual(state['phase'], 'PPO điều hướng')
        self.assertEqual(len(state['users']), 4)
        self.assertTrue(all(u['snr_db'] is None for u in state['users']))
        self.assertIsNotNone(state['target'])

    def test_http_routes_and_shutdown_without_opening_sockets(self):
        live = LiveDashboard()
        live.publish(sample())
        with patch('live_dashboard.ThreadingHTTPServer') as server, patch('live_dashboard.Thread') as thread:
            server.return_value.server_address = ('127.0.0.1', 8765)
            with contextlib.redirect_stdout(io.StringIO()): live.start()
            self.assertEqual(server.call_args.args[0], ('127.0.0.1', 8765))
            handler_class = server.call_args.args[1]
            for path in ['/', '/app.js', '/style.css', '/api/state', '/../../README.md']:
                handler = object.__new__(handler_class)
                handler.path = path
                handler.wfile = io.BytesIO()
                statuses = []
                handler.send_response = statuses.append
                handler.send_header = lambda *args: None
                handler.end_headers = lambda: None
                handler.send_error = statuses.append
                handler.do_GET()
                self.assertEqual(statuses, [404 if path.startswith('/..') else 200])
                if path == '/api/state':
                    self.assertEqual(json.loads(handler.wfile.getvalue())['state']['sim_time'], 1.)
            live.close()
            server.return_value.shutdown.assert_called_once()
            server.return_value.server_close.assert_called_once()
            thread.return_value.join.assert_called_once()


if __name__ == '__main__':
    unittest.main()
