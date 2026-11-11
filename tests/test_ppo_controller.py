import contextlib
import io
import json
import runpy
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
import dieukhien

USERS = np.array([[-100, -50, 0], [50, -50, 0], [100, 100, 0], [2, 0, 0]], dtype=float)

class Field:
    def __init__(self): self.pos = [0., 0., TARGET_ALTITUDE]
    def getSFVec3f(self): return list(self.pos)
    def setSFVec3f(self, pos): self.pos = list(pos)

class Node:
    def __init__(self, pos=None): self.pos = pos; self.field = Field()
    def getPosition(self): return self.pos
    def getField(self, name): return self.field
    def resetPhysics(self): pass

class Robot:
    def __init__(self, steps=251):
        self.remaining = steps; self.uav = Node()
        self.nodes = {name: Node(pos.tolist()) for name, pos in zip(USER_DEFS, USERS)}
    def getBasicTimeStep(self): return 8
    def getSelf(self): return self.uav
    def getFromDef(self, name): return self.nodes.get(name)
    def getDevice(self, name): return None
    def step(self, dt):
        self.remaining -= 1
        return 0 if self.remaining >= 0 else -1

class Socket:
    def __init__(self, packets=()): self.packets=list(packets); self.sent=[]
    def recvfrom(self, size):
        if not self.packets: raise BlockingIOError
        return self.packets.pop(0), ('127.0.0.1',1234)
    def sendto(self, data, addr): self.sent.append((json.loads(data),addr))

class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.bridge=PPOBridge(FOLDER)
        self.addCleanup(self.bridge.close)

    def test_full_observation_contract(self):
        self.bridge.start([0,0,TARGET_ALTITUDE], USERS)
        obs=self.bridge.observation([0,0,TARGET_ALTITUDE], USERS)
        np.testing.assert_allclose(obs[:6], [0,0,0,0,1,1])
        np.testing.assert_allclose(obs[6:14], (USERS[:,:2]/1000).ravel(), atol=1e-7)
        self.assertTrue(np.all(obs[14:]>0))
        self.assertTrue(np.isfinite(obs).all())
        self.assertFalse(self.bridge.vec.training)

    def test_action_is_acceleration_and_limits_are_applied(self):
        self.bridge.start([0,0,TARGET_ALTITUDE], USERS)
        with patch.object(self.bridge.model,'predict',return_value=(np.array([[1.,0.]]),None)):
            pos=np.array([0.,0.,TARGET_ALTITUDE])
            for expected_velocity in (2,4,5,5,5):
                new,_,_=self.bridge.next_target(pos, USERS)
                self.assertAlmostEqual(new[0]-pos[0],expected_velocity)
                pos=new
        self.assertLess(self.bridge.env.E,40000)
        self.assertEqual(self.bridge.env.t,5)

    def test_boundary_and_episode_end(self):
        self.bridge.start([499,0,TARGET_ALTITUDE], USERS)
        self.bridge.env.t=199
        with patch.object(self.bridge.model,'predict',return_value=(np.array([[1.,0.]]),None)):
            target,_,_=self.bridge.next_target([499,0,TARGET_ALTITUDE],USERS)
        self.assertEqual(target[0],500)
        self.assertEqual(self.bridge.env.vel[0],0)
        self.assertTrue(self.bridge.done)
        with self.assertRaises(RuntimeError):self.bridge.next_target(target,USERS)

    def test_actual_checkpoint_drives_controller_at_one_second_cadence(self):
        robot=Robot(251)  # one start step + two full 125-step model transitions
        with contextlib.redirect_stdout(io.StringIO()) as log:
            dieukhien.run(robot,self.bridge,Socket())
        self.assertEqual(self.bridge.env.t,2)
        self.assertIn('[PPO ACTIVE]',log.getvalue())
        self.assertIn('[PPO STEP 002]',log.getvalue())
        self.assertGreater(np.linalg.norm(robot.uav.field.pos[:2]),0.01)
        np.testing.assert_allclose(robot.uav.field.pos, self.bridge.env.pos-[500,500,0],atol=1e-8)
        self.assertEqual(robot.uav.field.pos[2],TARGET_ALTITUDE)

    def test_takeoff_finishes_before_first_ppo_action(self):
        robot=Robot(1);robot.uav.field.pos=[0,0,TARGET_ALTITUDE - 0.02]
        with contextlib.redirect_stdout(io.StringIO()):dieukhien.run(robot,self.bridge,Socket())
        self.assertEqual(robot.uav.field.pos[2],TARGET_ALTITUDE)
        self.assertTrue(self.bridge.started)
        self.assertEqual(self.bridge.env.t,0)

    def test_bad_packets_do_not_block_valid_relay(self):
        packet={'sender':'E_PUCK_CLIENT_2','seq':1}
        sock=Socket([b'bad json', b'[]', b'{"sender":[]}',json.dumps(packet).encode()])
        with contextlib.redirect_stdout(io.StringIO()):
            dieukhien.relay_packets(sock,np.array([50, -50,100]),Robot().nodes)
        self.assertEqual([addr[1] for _,addr in sock.sent],[9000,8080])
        self.assertEqual(sock.sent[0][0]['distance_to_drone'],100)
        self.assertEqual(sock.sent[0][0]['via_relay'],'Drone_PPO')
        self.assertEqual(sock.sent[0][0]['relay_node'],'Drone_PPO')

    def test_all_three_clients_in_range_at_lower_initial_altitude(self):
        robot=Robot()
        packets=[json.dumps({'sender':name,'seq':1}).encode() for name in USER_DEFS[:3]]
        sock=Socket(packets)
        with contextlib.redirect_stdout(io.StringIO()):
            dieukhien.relay_packets(sock,np.array([0,0,TARGET_ALTITUDE]),robot.nodes)
        self.assertEqual(len(sock.sent),6)
        self.assertEqual({p['sender'] for p,_ in sock.sent},set(USER_DEFS[:3]))
        self.bridge.start([0,0,TARGET_ALTITUDE],USERS)
        self.assertEqual(self.bridge.env.H,TARGET_ALTITUDE)
        target,_,_=self.bridge.next_target([0,0,TARGET_ALTITUDE],USERS)
        self.assertEqual(target[2],TARGET_ALTITUDE)

    def test_missing_node_and_wrong_user_count_fail_explicitly(self):
        robot=Robot();del robot.nodes[USER_DEFS[0]]
        with self.assertRaisesRegex(RuntimeError,USER_DEFS[0]):dieukhien.find_users(robot)
        with self.assertRaises(ValueError):self.bridge.start([0,0,TARGET_ALTITUDE],USERS[:3])

    def test_client_transmissions_use_simulation_time(self):
        class ClientRobot(Robot):
            def __init__(self):
                super().__init__(steps=501)
                self.sim_time=0.0
            def getTime(self): return self.sim_time
            def step(self, dt):
                self.sim_time += dt/1000
                return super().step(dt)
        controller=types.ModuleType('controller')
        controller.Robot=ClientRobot
        for name,sender in zip(['epuck_ns3_ping','epuck_ns3_ping1','epuck_ns3_ping2'],USER_DEFS[:3]):
            sock=Socket()
            with patch.dict(sys.modules,{'controller':controller}), patch('socket.socket',return_value=sock), contextlib.redirect_stdout(io.StringIO()):
                runpy.run_path(str(FOLDER.parent/name/(name+'.py')))
            self.assertEqual([packet['seq'] for packet,_ in sock.sent],[1,2,3])
            self.assertTrue(all(packet['sender']==sender and addr[1]==8081 for packet,addr in sock.sent))

    def test_link_status_separates_missing_packets_from_range(self):
        with contextlib.redirect_stdout(io.StringIO()) as log:
            dieukhien.log_links(np.array([0,0,30]),Robot().nodes,{'E_PUCK_CLIENT_1':9.,'E_PUCK_CLIENT_2':1.},10.)
        self.assertIn('IN_RANGE RX_OK',log.getvalue())
        self.assertIn('IN_RANGE RX_STALE',log.getvalue())
        self.assertIn('IN_RANGE NO_PACKETS_YET',log.getvalue())

    def test_trained_policy_keeps_all_three_clients_for_full_episode(self):
        pos=np.array([0.005307792794693136, -1.8952631440755435e-7, TARGET_ALTITUDE])
        self.bridge.start(pos, USERS)
        for _ in range(200):
            target,_,_=self.bridge.next_target(pos,USERS)
            samples=pos+(target-pos)*np.arange(1,126)[:,None]/125
            distances=np.linalg.norm(samples[:,None,:]-USERS[None,:3,:],axis=2)
            self.assertTrue(np.all(distances<=150),f'Outage at step {self.bridge.env.t}')
            pos=target
        self.assertTrue(self.bridge.done)

    def test_worlds_have_named_users_and_correct_controller(self):
        for name,controller in [('VD2_vscode.wbt','<extern>'),('VD2.wbt','dieukhien')]:
            s=(ROOT/'drone-rl/webots_world/worlds'/name).read_text()
            for node in USER_DEFS:self.assertEqual(s.count(f'DEF {node} E-puck'),1)
            self.assertIn(f'controller "{controller}"',s)
            self.assertIn('/E-puck.proto"',s)

if __name__=='__main__':unittest.main()
