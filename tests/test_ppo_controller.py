import contextlib
import io
import json
import runpy
import re
import types
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'drone-rl/webots_world/controllers/dieukhien'
sys.path.insert(0, str(FOLDER))
from ppo_bridge import PPOBridge, USER_DEFS, TARGET_ALTITUDE
from ns3_cosim import Ns3CoSimClient
import dieukhien

USERS = np.array([[-100, -50, 0], [50, -50, 0], [100, 100, 0], [2, 0, 0]], dtype=float)


class Node:
    def __init__(self, pos=None): self.pos = pos
    def getPosition(self): return self.pos


class Device:
    def __init__(self, values=(0., 0., 0.)):
        self.values = list(values)
        self.commands = []
    def enable(self, timestep): pass
    def getValues(self): return list(self.values)
    def getRollPitchYaw(self): return list(self.values)
    def setPosition(self, value): pass
    def setVelocity(self, value): self.commands.append(value)


class Robot:
    # Deliberately exposes no Supervisor APIs.
    def __init__(self, steps=251):
        self.remaining = steps
        self.nodes = {name: Node(pos.tolist()) for name, pos in zip(USER_DEFS, USERS)}
        self.devices = {name: Device() for name in (
            'inertial unit', 'gyro', 'front left propeller', 'front right propeller',
            'rear left propeller', 'rear right propeller')}
        self.devices['gps'] = Device((0., 0., TARGET_ALTITUDE))
    def getBasicTimeStep(self): return 8
    def getCustomData(self):
        return json.dumps({'users_pos': {name: node.pos for name, node in self.nodes.items()}})
    def getDevice(self, name): return self.devices.get(name)
    def step(self, dt):
        self.remaining -= 1
        return 0 if self.remaining >= 0 else -1


class Socket:
    def __init__(self, packets=()): self.packets = list(packets); self.sent = []
    def recvfrom(self, size):
        if not self.packets: raise BlockingIOError
        return self.packets.pop(0), ('127.0.0.1', 1234)
    def sendto(self, data, addr): self.sent.append((json.loads(data), addr))


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Spawn a single real C++ ns-3.47 server instance shared across tests in this class
        cls.ns3_client = Ns3CoSimClient(port=0, auto_spawn=True, quiet_spawn=True)

    @classmethod
    def tearDownClass(cls):
        cls.ns3_client.close()

    def setUp(self):
        self.ns3_client.reset_stats(shadow_loss_db=[0.0, 0.0, 0.0, 0.0])
        self.bridge = PPOBridge(FOLDER, use_ns3=True, ns3_client=self.ns3_client)
        self.addCleanup(self.bridge.close)

    def test_full_observation_contract(self):
        self.bridge.start([0, 0, TARGET_ALTITUDE], USERS)
        obs = self.bridge.observation([0, 0, TARGET_ALTITUDE], USERS)
        np.testing.assert_allclose(obs[:6], [0, 0, 0, 0, 1, 1])
        np.testing.assert_allclose(obs[6:14], (USERS[:, :2] / 1000).ravel(), atol=1e-7)
        self.assertTrue(np.all(obs[14:] > 0))
        self.assertTrue(np.isfinite(obs).all())
        self.assertFalse(self.bridge.vec.training)
        # Verify obs[14:18] comes from ns-3 spectral_eff
        ns3_spec = np.asarray(self.bridge.last_ns3_feedback['spectral_eff'], dtype=np.float32) / 10.0
        np.testing.assert_allclose(obs[14:18], ns3_spec, rtol=1e-5)

    def test_action_is_acceleration_and_limits_are_applied(self):
        self.bridge.ns3_control_assist = False
        self.bridge.start([0, 0, TARGET_ALTITUDE], USERS)
        with patch.object(self.bridge.model, 'predict', return_value=(np.array([[1., 0.]]), None)):
            pos = np.array([0., 0., TARGET_ALTITUDE])
            for expected_velocity in (2, 4, 5, 5, 5):
                new, _, _ = self.bridge.next_target(pos, USERS)
                self.assertAlmostEqual(new[0] - pos[0], expected_velocity)
                pos = new
        self.assertLess(self.bridge.env.E, 40000)
        self.assertEqual(self.bridge.env.t, 5)

    def test_boundary_and_episode_end(self):
        self.bridge.ns3_control_assist = False
        self.bridge.start([499, 0, TARGET_ALTITUDE], USERS)
        self.bridge.env.t = 199
        with patch.object(self.bridge.model, 'predict', return_value=(np.array([[1., 0.]]), None)):
            target, _, _ = self.bridge.next_target([499, 0, TARGET_ALTITUDE], USERS)
        self.assertEqual(target[0], 500)
        self.assertEqual(self.bridge.env.vel[0], 0)
        self.assertTrue(self.bridge.done)
        with self.assertRaises(RuntimeError): self.bridge.next_target(target, USERS)

    def test_actual_checkpoint_drives_controller_at_one_second_cadence(self):
        robot = Robot(251)  # one start step + two full 125-step model transitions
        with contextlib.redirect_stdout(io.StringIO()) as log:
            dieukhien.run(robot, self.bridge, Socket())
        self.assertEqual(self.bridge.env.t, 2)
        self.assertIn('[PPO ACTIVE]', log.getvalue())
        self.assertIn('[PPO STEP 002]', log.getvalue())
        self.assertIn('ns3_min_snr=', log.getvalue())
        for name in ('front left propeller', 'front right propeller',
                     'rear left propeller', 'rear right propeller'):
            commands = robot.devices[name].commands
            self.assertEqual(len(commands), 252)
            self.assertTrue(np.isfinite(commands).all())
            self.assertGreater(max(np.abs(commands)), 60)
        self.assertIn('flight_mode=PHYSICS_PID_4ROTOR', log.getvalue())




    def test_bad_packets_do_not_block_valid_relay(self):
        packet = {'sender': 'E_PUCK_CLIENT_2', 'seq': 1}
        sock = Socket([b'bad json', b'[]', b'{"sender":[]}', json.dumps(packet).encode()])
        with contextlib.redirect_stdout(io.StringIO()):
            dieukhien.relay_packets(sock, np.array([50, -50, 100]), Robot().nodes)
        self.assertEqual([addr[1] for _, addr in sock.sent], [9000, 8080])
        self.assertEqual(sock.sent[0][0]['distance_to_drone'], 100)
        self.assertEqual(sock.sent[0][0]['via_relay'], 'Drone_PPO')
        self.assertEqual(sock.sent[0][0]['relay_node'], 'Drone_PPO')

    def test_all_three_clients_in_range_at_lower_initial_altitude(self):
        robot = Robot()
        packets = [json.dumps({'sender': name, 'seq': 1}).encode() for name in USER_DEFS[:3]]
        sock = Socket(packets)
        with contextlib.redirect_stdout(io.StringIO()):
            dieukhien.relay_packets(sock, np.array([0, 0, TARGET_ALTITUDE]), robot.nodes)
        self.assertEqual(len(sock.sent), 6)
        self.assertEqual({p['sender'] for p, _ in sock.sent}, set(USER_DEFS[:3]))
        self.bridge.start([0, 0, TARGET_ALTITUDE], USERS)
        self.assertEqual(self.bridge.env.H, TARGET_ALTITUDE)
        target, _, _ = self.bridge.next_target([0, 0, TARGET_ALTITUDE], USERS)
        self.assertEqual(target[2], TARGET_ALTITUDE)


    def test_client_transmissions_use_simulation_time(self):
        class ClientRobot(Robot):
            def __init__(self):
                super().__init__(steps=501)
                self.sim_time = 0.0
            def getTime(self): return self.sim_time
            def step(self, dt):
                self.sim_time += dt / 1000
                return super().step(dt)
        controller = types.ModuleType('controller')
        controller.Robot = ClientRobot
        for name, sender in zip(['epuck_ns3_ping', 'epuck_ns3_ping1', 'epuck_ns3_ping2'], USER_DEFS[:3]):
            sock = Socket()
            with patch.dict(sys.modules, {'controller': controller}), patch('socket.socket', return_value=sock), contextlib.redirect_stdout(io.StringIO()):
                runpy.run_path(str(FOLDER.parent / name / (name + '.py')))
            self.assertEqual([packet['seq'] for packet, _ in sock.sent], [1, 2, 3])
            self.assertTrue(all(packet['sender'] == sender and addr[1] == 8081 for packet, addr in sock.sent))

    def test_link_status_separates_missing_packets_from_range(self):
        with contextlib.redirect_stdout(io.StringIO()) as log:
            dieukhien.log_links(np.array([0, 0, 30]), Robot().nodes, {'E_PUCK_CLIENT_1': 9., 'E_PUCK_CLIENT_2': 1.}, 10.)
        self.assertIn('IN_RANGE RX_OK', log.getvalue())
        self.assertIn('IN_RANGE RX_STALE', log.getvalue())
        self.assertIn('IN_RANGE NO_PACKETS_YET', log.getvalue())

    def test_trained_policy_keeps_all_three_clients_for_full_episode(self):
        pos = np.array([0.005307792794693136, -1.8952631440755435e-7, TARGET_ALTITUDE])
        self.bridge.start(pos, USERS)
        for _ in range(200):
            target, _, info = self.bridge.next_target(pos, USERS)
            self.assertTrue(info['ns3_active'])
            samples = pos + (target - pos) * np.arange(1, 126)[:, None] / 125
            distances = np.linalg.norm(samples[:, None, :] - USERS[None, :3, :], axis=2)
            self.assertTrue(np.all(distances <= 150), f'Outage at step {self.bridge.env.t}')
            pos = target
        self.assertTrue(self.bridge.done)


    def test_ns3_closed_loop_verifies_two_hop_packets_and_drops_server_outage(self):
        robot = Robot()
        # Case 1: UAV at origin (0, 0, 30) -> all 3 clients and server within 150m -> ns-3 delivers all 3
        packets = [json.dumps({'sender': name, 'seq': 1, 'payload': 'hello'}).encode() for name in USER_DEFS[:3]]
        sock = Socket(packets)
        with contextlib.redirect_stdout(io.StringIO()):
            dieukhien.relay_packets(sock, np.array([0., 0., 30.]), robot.nodes, sim_time=2.0, ns3_client=self.ns3_client)
        self.assertEqual(len(sock.sent), 3)
        for pkt, addr in sock.sent:
            self.assertEqual(addr[1], 9000)
            self.assertTrue(pkt['ns3_verified'])
            self.assertGreater(pkt['ns3_e2e_delay_ms'], 0.0)
            self.assertEqual(pkt['ns3_pdr'], 1.0)

        # Case 2: UAV near Client 1 (-100, -50, 30), but move Server far away (300, 300, 0)
        # Verify ns-3 drops the packet due to DOWNLINK_OUT_OF_RANGE (2nd hop check!)
        robot.nodes['E_PUCK_SERVER'].pos = [300., 300., 0.]
        sock2 = Socket([json.dumps({'sender': 'E_PUCK_CLIENT_1', 'seq': 2}).encode()])
        with contextlib.redirect_stdout(io.StringIO()) as log:
            dieukhien.relay_packets(sock2, np.array([-100., -50., 30.]), robot.nodes, sim_time=4.0, ns3_client=self.ns3_client)
        self.assertEqual(len(sock2.sent), 0)
        self.assertIn('DOWNLINK_OUT_OF_RANGE', log.getvalue())

    def test_ns3_obstacle_shadowing_steers_uav_to_recover_degraded_link(self):
        # Apply 4.0 dB obstacle shadowing on Client 3 in ns-3 so at PPO's open-loop equilibrium
        # (~128.03 m from Client 3, unshadowed SNR 12.66 dB), Client 3 remains in SNR outage (8.66 dB < 10.35 dB).
        pos_closed_loop = np.array([0.0, 0.0, TARGET_ALTITUDE])
        self.bridge.use_ns3 = True
        self.bridge.ns3_control_assist = True
        self.bridge.start(pos_closed_loop, USERS, shadow_loss_db=[0.0, 0.0, 4.0, 0.0])

        initial_fb = self.bridge.last_ns3_feedback
        self.assertFalse(initial_fb['link_ok'][2])  # Client 3 initially suffers SNR outage in ns-3
        self.assertGreater(initial_fb['assist_weight'], 0.0)

        # Run 50 steps with ns-3 closed-loop participation (obs[14:18] + PD control assist)
        for _ in range(50):
            pos_closed_loop, _, info = self.bridge.next_target(pos_closed_loop, USERS)

        # Compare against open-loop without ns-3 (old PBL3-LTM analytical channel)
        pos_open_loop = np.array([0.0, 0.0, TARGET_ALTITUDE])
        self.bridge.use_ns3 = False
        self.bridge.start(pos_open_loop, USERS)
        for _ in range(50):
            pos_open_loop, _, _ = self.bridge.next_target(pos_open_loop, USERS)
        open_loop_fb = self.ns3_client.query_state(pos_open_loop, USERS, sim_time=50.0)

        dist_c3_closed = float(np.linalg.norm(pos_closed_loop - USERS[2]))
        dist_c3_open = float(np.linalg.norm(pos_open_loop - USERS[2]))
        self.assertLess(dist_c3_closed, dist_c3_open - 12.0)
        self.assertFalse(open_loop_fb['link_ok'][2])  # Without ns-3 in loop, Client 3 stays in outage (8.66 dB)
        self.assertTrue(all(info['ns3_link_ok']))     # With ns-3 in loop, ALL 4 links are OK!


class OfflineControllerTests(unittest.TestCase):
    def test_policy_cadence_and_motor_commands_without_supervisor(self):
        robot = Robot(251)
        with contextlib.redirect_stdout(io.StringIO()):
            dieukhien.run(robot, self.bridge, Socket())
        self.assertEqual(self.bridge.env.t, 2)
        for name in ('front left propeller', 'front right propeller',
                     'rear left propeller', 'rear right propeller'):
            commands = robot.devices[name].commands
            self.assertEqual(len(commands), 252)
            self.assertTrue(np.isfinite(commands).all())
            self.assertGreater(max(np.abs(commands)), 60)

    def setUp(self):
        self.bridge = PPOBridge(FOLDER, use_ns3=False)
        self.addCleanup(self.bridge.close)

    def test_takeoff_finishes_before_first_ppo_action(self):
        robot = Robot(1)
        robot.devices['gps'].values[2] = TARGET_ALTITUDE - 2
        with contextlib.redirect_stdout(io.StringIO()): dieukhien.run(robot, self.bridge, Socket())
        self.assertFalse(self.bridge.started)
        robot.remaining = 1
        robot.devices['gps'].values[2] = TARGET_ALTITUDE - 0.02
        with contextlib.redirect_stdout(io.StringIO()): dieukhien.run(robot, self.bridge, Socket())
        self.assertTrue(self.bridge.started)
        self.assertEqual(self.bridge.env.t, 0)

    def test_missing_sensors_and_kinematic_mode_fail_explicitly(self):
        robot = Robot()
        del robot.devices['gps']
        with self.assertRaisesRegex(RuntimeError, 'GPS'):
            dieukhien.run(robot, self.bridge, Socket())
        with self.assertRaisesRegex(ValueError, 'supervisor FALSE'):
            dieukhien.run(Robot(), self.bridge, Socket(), force_mode='kinematic')

    def test_invalid_gps_does_not_start_policy(self):
        robot = Robot(2)
        robot.devices['gps'].values = [float('nan')] * 3
        with contextlib.redirect_stdout(io.StringIO()): dieukhien.run(robot, self.bridge, Socket())
        self.assertFalse(self.bridge.started)


    def test_missing_node_and_wrong_user_count_fail_explicitly(self):
        robot = Robot(); del robot.nodes[USER_DEFS[0]]
        with self.assertRaisesRegex(RuntimeError, USER_DEFS[0]): dieukhien.find_users(robot)
        with self.assertRaises(ValueError): self.bridge.start([0, 0, TARGET_ALTITUDE], USERS[:3])

    def test_worlds_have_named_users_and_correct_controller(self):
        for name, controller in [('VD2_vscode.wbt', '<extern>'), ('VD2.wbt', 'dieukhien')]:
            s = (ROOT / 'drone-rl/webots_world/worlds' / name).read_text()
            for node in USER_DEFS: self.assertEqual(s.count(f'DEF {node} E-puck'), 1)
            self.assertTrue(f'controller "{controller}"' in s or 'controller "dieukhien"' in s)
            self.assertIn('supervisor FALSE', s)
            config = json.loads(json.loads(re.search(r'customData ("(?:\\.|[^"\\])*")', s)[1]))
            for node in USER_DEFS:
                body = s.split(f'DEF {node} E-puck {{', 1)[1].split('}', 1)[0]
                position = list(map(float, re.search(r'translation ([^\n]+)', body)[1].split()))
                np.testing.assert_allclose(config['users_pos'][node], position)

            self.assertIn('/E-puck.proto"', s)


if __name__ == '__main__':
    unittest.main()
