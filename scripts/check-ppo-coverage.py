"""Closed-loop PPO + ns-3.47 coverage, PDR, delay, throughput and obstacle diagnostic."""
from pathlib import Path
import sys
import json
import re
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'drone-rl/webots_world/controllers/dieukhien'
sys.path.insert(0, str(FOLDER))
from ppo_bridge import PPOBridge, USER_DEFS
from ns3_cosim import Ns3CoSimClient
from dieukhien import COMM_RANGE_DRONE

world = (ROOT / 'drone-rl/webots_world/worlds/VD2_vscode.wbt').read_text()


def position(block):
    return np.array([float(x) for x in re.search(r'\btranslation\s+([^\n]+)', block).group(1).split()])


users = np.array([position(world.split('DEF ' + name + ' E-puck {', 1)[1].split('}', 1)[0]) for name in USER_DEFS])
initial = position(world.split('\nMavic2Pro {', 1)[1].split('}', 1)[0])


def run_episode(ns3_client, use_ns3=True, ns3_assist=True, shadow_loss_db=(0.0, 0.0, 0.0, 0.0), altitude=30.0):
    bridge = PPOBridge(
        FOLDER,
        altitude=altitude,
        use_ns3=use_ns3,
        ns3_control_assist=ns3_assist,
        ns3_client=ns3_client,
    )
    try:
        pos = initial.copy()
        pos[2] = altitude
        ns3_client.reset_stats(shadow_loss_db=shadow_loss_db)
        bridge.start(pos, users, shadow_loss_db=shadow_loss_db)
        start_dist = np.linalg.norm(users[:3] - pos, axis=1)
        coverage = []
        first_drop = None
        first_all_out = None
        last_verdict = None
        seq = 0
        while not bridge.done:
            step_start_time = float(bridge.env.t)
            # Clients transmit at t = 0, 2, 4, ..., 198 s (every 2 simulation seconds)
            if int(round(step_start_time)) % 2 == 0:
                seq += 1
                for sender in USER_DEFS[:3]:
                    pkt = {
                        'type': 'DATA_PACKET',
                        'sender': sender,
                        'destination': 'SERVER_DEST',
                        'seq': seq,
                        'payload': f'Hello from {sender} via Drone Relay',
                    }
                    last_verdict = ns3_client.evaluate_packet(pkt, pos, users, sim_time=step_start_time)

            target, _, _ = bridge.next_target(pos, users)
            # Match the controller's 125 interpolated states per 1 s action.
            for tick in range(1, 126):
                point = pos + (target - pos) * (tick / 125)
                ok = np.linalg.norm(users[:3] - point, axis=1) <= COMM_RANGE_DRONE
                t = (bridge.env.t - 1) + tick * 0.008
                coverage.append(ok)
                if first_drop is None and not ok.all():
                    first_drop = t
                if first_all_out is None and not ok.any():
                    first_all_out = t
            pos = target

        c = np.array(coverage)
        final_fb = ns3_client.query_state(pos, users, sim_time=float(bridge.env.t))
        return dict(
            altitude_m=altitude,
            use_ns3=use_ns3,
            ns3_control_assist=ns3_assist,
            shadow_loss_db=list(shadow_loss_db),
            initial_distances_m=np.round(start_dist, 3).tolist(),
            initial_in_range=(start_dist <= COMM_RANGE_DRONE).tolist(),
            duration_s=bridge.env.t,
            per_client_in_range_pct=np.round(100 * c.mean(axis=0), 3).tolist(),
            all_three_in_range_pct=round(float(100 * c.all(axis=1).mean()), 4),
            all_three_out_of_range_pct=round(float(100 * (~c.any(axis=1)).mean()), 4),
            first_any_out_of_range_s=first_drop,
            first_all_out_of_range_s=first_all_out,
            ns3_tx_packets=int(last_verdict.get('tx_packets', 0)) if last_verdict else 0,
            ns3_rx_packets=int(last_verdict.get('rx_packets', 0)) if last_verdict else 0,
            ns3_pdr_pct=round(100.0 * float(last_verdict.get('pdr', 1.0)), 2) if last_verdict else 100.0,
            ns3_avg_delay_ms=round(float(last_verdict.get('avg_delay_ms', 0.0)), 3) if last_verdict else 0.0,
            ns3_throughput_bps=round(float(last_verdict.get('throughput_bps', 0.0)), 2) if last_verdict else 0.0,
            final_ns3_snr_db=np.round(final_fb.get('snr_db', []), 2).tolist(),
            final_ns3_link_ok=final_fb.get('link_ok', []),
        )
    finally:
        bridge.close()


ns3_client = Ns3CoSimClient(port=0, auto_spawn=True, quiet_spawn=True)
try:
    nominal_closed_loop = run_episode(ns3_client, use_ns3=True, ns3_assist=True, shadow_loss_db=(0., 0., 0., 0.))
    obstacle_open_loop = run_episode(ns3_client, use_ns3=False, ns3_assist=False, shadow_loss_db=(0., 0., 4.0, 0.))
    obstacle_closed_loop = run_episode(ns3_client, use_ns3=True, ns3_assist=True, shadow_loss_db=(0., 0., 4.0, 0.))
finally:
    ns3_client.close()

print(json.dumps({
    'scope': 'closed-loop PPO + ns-3.47 co-simulation rollout (8 ms interpolation + UDP packet evaluation)',
    'radius_m': COMM_RANGE_DRONE,
    'results': [nominal_closed_loop],
    'obstacle_comparison_4dB_client3': {
        'open_loop_without_ns3': obstacle_open_loop,
        'closed_loop_with_ns3': obstacle_closed_loop,
    },
}, ensure_ascii=False, indent=2))
