"""Webots UAV relay with 6-DOF rotor physics control, closed-loop ns-3.47 feedback and PPO."""
from pathlib import Path
import json
import os
import socket
import numpy as np
from ppo_bridge import PPOBridge, USER_DEFS, POLICY_DT
from flight_physics import MavicPhysicsPilot
from live_dashboard import LiveDashboard

MODEL_DIR = Path(__file__).resolve().parent
COMM_RANGE_DRONE = 150.0


class FixedUser:
    """Known position of a stationary E-puck, supplied by world customData."""

    def __init__(self, position):
        self.position = position

    def getPosition(self):
        return list(self.position)


def find_users(robot):
    try:
        config = json.loads(robot.getCustomData())
        positions = config['users_pos']
        nodes = {}
        for name in USER_DEFS:
            pos = np.asarray(positions[name], dtype=float)
            if pos.shape != (3,) or not np.isfinite(pos).all():
                raise ValueError(f'Tọa độ không hợp lệ: {name}')
            nodes[name] = FixedUser(pos.tolist())
        return nodes
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f'customData cần users_pos chứa tọa độ XYZ của 4 E-puck: {exc}') from exc


def relay_packets(sock, position, nodes, received_at=None, sim_time=0.0, ns3_client=None, live=None):
    # Bounded draining: a packet burst must not starve flight updates.
    for _ in range(32):
        try:
            data, _ = sock.recvfrom(2048)
        except BlockingIOError:
            break
        try:
            packet = json.loads(data.decode('utf-8'))
            if not isinstance(packet, dict):
                raise ValueError('JSON phải là object')
            sender = packet.get('sender')
            if not isinstance(sender, str) or sender not in USER_DEFS[:3]:
                raise ValueError('sender không thuộc ba client đã cấu hình')
        except (UnicodeError, ValueError) as exc:
            print(f'[UDP DROP] Gói không hợp lệ: {exc}', flush=True)
            continue
        if received_at is not None:
            received_at[sender] = sim_time
        distance = float(np.linalg.norm(position - np.asarray(nodes[sender].getPosition())))
        server_node = nodes.get(USER_DEFS[3])
        server_distance = (
            float(np.linalg.norm(position - np.asarray(server_node.getPosition())))
            if server_node is not None else 0.0
        )

        if ns3_client is not None:
            users_pos = [nodes[name].getPosition() for name in USER_DEFS]
            verdict = ns3_client.evaluate_packet(packet, position, users_pos, sim_time=sim_time)
            if not verdict.get('forward_allowed', False):
                reason = verdict.get('drop_reason', 'NS3_LINK_OUTAGE')
                if live is not None:
                    live.record_packet(sender, packet.get('seq'), False, sim_time, reason, verdict)
                print(f'[UDP DROP][NS-3] {sender} seq={packet.get("seq")} '
                      f'd_up={distance:.2f}m d_srv={server_distance:.2f}m reason={reason}',
                      flush=True)
                continue
            packet.update(
                via_relay='Drone_PPO',
                relay_node='Drone_PPO',
                distance_to_drone=round(distance, 2),
                distance_to_server=round(server_distance, 2),
                ns3_verified=True,
                ns3_snr_up_db=round(float(verdict.get('snr_up_db', 0.0)), 2),
                ns3_snr_down_db=round(float(verdict.get('snr_down_db', 0.0)), 2),
                ns3_e2e_delay_ms=round(float(verdict.get('e2e_delay_ms', 0.0)), 3),
                ns3_pdr=round(float(verdict.get('pdr', 1.0)), 4),
                ns3_throughput_bps=round(float(verdict.get('throughput_bps', 0.0)), 2),
            )
            payload = json.dumps(packet).encode('utf-8')
            sock.sendto(payload, ('127.0.0.1', 9000))
            if live is not None:
                live.record_packet(sender, packet.get('seq'), True, sim_time, verdict=verdict)
            print(f'[UDP RELAY][NS-3] {sender} seq={packet.get("seq")} '
                  f'd_up={distance:.2f}m d_srv={server_distance:.2f}m '
                  f'SNR={packet["ns3_snr_up_db"]:.1f}/{packet["ns3_snr_down_db"]:.1f}dB '
                  f'delay={packet["ns3_e2e_delay_ms"]:.2f}ms PDR={100*packet["ns3_pdr"]:.1f}%',
                  flush=True)
            continue

        if distance > COMM_RANGE_DRONE or server_distance > COMM_RANGE_DRONE:
            if live is not None:
                live.record_packet(sender, packet.get('seq'), False, sim_time, 'OUT_OF_RANGE')
            print(f'[UDP DROP] {sender} seq={packet.get("seq")} '
                  f'd={distance:.2f} m d_srv={server_distance:.2f} m', flush=True)
            continue
        packet.update(via_relay='Drone_PPO', relay_node='Drone_PPO',
                      distance_to_drone=round(distance, 2),
                      distance_to_server=round(server_distance, 2))
        payload = json.dumps(packet).encode('utf-8')
        for port in (9000, 8080):
            sock.sendto(payload, ('127.0.0.1', port))
        if live is not None:
            live.record_packet(sender, packet.get('seq'), True, sim_time)
        print(f'[UDP RELAY] {sender} seq={packet.get("seq")} d={distance:.2f} m', flush=True)


def log_links(position, nodes, received_at, sim_time, ns3_feedback=None):
    """Report geometry, ns-3 SNR/link status, and reception status."""
    snr_list = ns3_feedback.get('snr_db', []) if isinstance(ns3_feedback, dict) else []
    link_ok_list = ns3_feedback.get('link_ok', []) if isinstance(ns3_feedback, dict) else []
    for idx, sender in enumerate(USER_DEFS[:3]):
        distance = float(np.linalg.norm(position - np.asarray(nodes[sender].getPosition())))
        in_range = distance <= COMM_RANGE_DRONE
        if idx < len(link_ok_list):
            in_range = in_range and bool(link_ok_list[idx])
        coverage = 'IN_RANGE' if in_range else 'OUT_OF_RANGE'
        last = received_at.get(sender)
        reception = ('NO_PACKETS_YET' if last is None else
                     'RX_OK' if sim_time - last <= 6.0 else 'RX_STALE')
        snr_str = f' snr={snr_list[idx]:.1f}dB' if idx < len(snr_list) else ''
        print(f'[LINK] {sender} d={distance:.2f}m{snr_str} {coverage} {reception}', flush=True)


def run(robot, bridge, sock, force_mode=None, live=None):
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0
    interval_steps = round(POLICY_DT / dt)
    if interval_steps < 1 or not np.isclose(interval_steps * dt, POLICY_DT):
        raise ValueError('basicTimeStep phải chia hết chu kỳ PPO 1000 ms (world dùng 8 ms).')
    mode_setting = (force_mode or os.environ.get('CONTROL_MODE', 'physics')).lower()
    if mode_setting not in ('auto', 'physics'):
        raise ValueError('supervisor FALSE chỉ hỗ trợ CONTROL_MODE=physics hoặc auto.')
    nodes = find_users(robot)
    users = lambda: [nodes[name].getPosition() for name in USER_DEFS]
    pilot = MavicPhysicsPilot(robot, timestep)
    if not pilot.available:
        raise RuntimeError('Bay vật lý cần GPS, inertial unit, gyro và đủ 4 động cơ.')

    print(f'[PPO READY] Model={MODEL_DIR / "ppo_relay_3epuck.zip"}; obs=18; action=2; '
          f'chu kỳ={POLICY_DT:g}s; CPU; ns3_closed_loop={bridge.use_ns3}; '
          f'flight_mode=PHYSICS_PID_4ROTOR',
          flush=True)
    print('[PPO CONFIG] 4 node = 3 client + server; xy mô hình = xy Webots + 500 m. '
          'Môi trường relay_3epuck_v1, ĐÃ ĐÓNG VÒNG PHẢN HỒI KÊNH & ĐIỀU KHIỂN VỚI NS-3.47.', flush=True)

    active = False
    phase = 0
    start = target = None
    takeoff_xy = None
    last_climb_log = -1
    ended_logged = False
    received_at = {}
    sim_time = 0.0
    next_link_log = 0.0

    while robot.step(timestep) != -1:
        sim_time += dt
        gps_vals = np.asarray(pilot.gps.getValues(), dtype=float)
        if not np.isfinite(gps_vals).all():
            continue  # Wait for a valid GPS sample before flight or network updates.
        pos = gps_vals
        if takeoff_xy is None:
            takeoff_xy = pos[:2].copy()

        relay_packets(sock, pos, nodes, received_at, sim_time, ns3_client=bridge.ns3, live=live)
        if live is not None and live.ready():
            feedback = (bridge.ns3.last_feedback if bridge.ns3 is not None else None) or bridge.last_ns3_feedback or {}
            snrs = feedback.get('snr_db', [])
            link_ok = feedback.get('link_ok', [])
            live.publish(dict(
                sim_time=sim_time, position=pos.tolist(), speed=float(np.linalg.norm(pilot.vel)),
                altitude=bridge.altitude, battery=float(bridge.env.E), policy_step=int(bridge.env.t),
                target=None if target is None else target.tolist(), ns3_enabled=bridge.use_ns3,
                phase='Cất cánh' if not active else 'Giữ vị trí' if bridge.done else 'PPO điều hướng',
                feedback=feedback,
                users=[dict(name=name, position=nodes[name].getPosition(),
                            distance=float(np.linalg.norm(pos - np.asarray(nodes[name].getPosition()))),
                            snr_db=snrs[i] if i < len(snrs) else None,
                            link_ok=bool(link_ok[i]) if i < len(link_ok) else None)
                       for i, name in enumerate(USER_DEFS)],
            ))
        if sim_time >= next_link_log:
            log_links(pos, nodes, received_at, sim_time, bridge.last_ns3_feedback)
            next_link_log = sim_time + 5.0

        if not active:
            pos = pilot.step_attitude_and_thrust(
                [takeoff_xy[0], takeoff_xy[1], bridge.altitude],
                target_vel_xy=[0.0, 0.0],
                sim_time=sim_time,
            )
            reached_altitude = abs(pos[2] - bridge.altitude) <= 0.45 and abs(pilot.vel[2]) <= 1.5
            height_band = int(pos[2] // 10)
            if height_band != last_climb_log:
                print(f'[TAKEOFF] z={pos[2]:.1f}/{bridge.altitude:g} m '
                      f'(PHYSICS)', flush=True)
                last_climb_log = height_band

            if reached_altitude:
                start_pos = np.array([pos[0], pos[1], bridge.altitude], dtype=float)
                bridge.start(start_pos, users())
                active = True
                print('[PPO ACTIVE] PPO + ns-3 tiếp quản chuyển động ngang.', flush=True)
            continue

        if phase == 0:
            if bridge.done:
                pilot.step_attitude_and_thrust(
                    [target[0], target[1], bridge.altitude],
                    target_vel_xy=[0.0, 0.0],
                    sim_time=sim_time,
                )
                if not ended_logged:
                    print('[PPO DONE] Hết episode/pin mô hình. Giữ vị trí; reset world để chạy lại.', flush=True)
                    ended_logged = True
                continue
            start = np.array([pos[0], pos[1], bridge.altitude], dtype=float)
            target, action, info = bridge.next_target(start, users())
            ns3_tag = ''
            if info.get('ns3_active'):
                min_snr = min(info.get('ns3_snr_db', [0.0]) or [0.0])
                w = info.get('ns3_assist_weight', 0.0)
                ns3_tag = f' ns3_min_snr={min_snr:.1f}dB ns3_assist={w:.2f}'
            print(f'[PPO STEP {bridge.env.t:03d}] action={action.round(4).tolist()} '
                  f'velocity={bridge.env.vel.round(3).tolist()} '
                  f'target={target.round(3).tolist()} actual_pos={pos.round(3).tolist()} '
                  f'battery={bridge.env.E:.1f} J{ns3_tag}',
                  flush=True)

        phase += 1
        # Interpolate the 1 s model transition over 125 Webots steps (8 ms).
        new_pos = start + (target - start) * (phase / interval_steps)
        ref_vel = (target[:2] - start[:2]) / POLICY_DT
        pilot.step_attitude_and_thrust(
            [new_pos[0], new_pos[1], bridge.altitude],
            target_vel_xy=ref_vel,
            sim_time=sim_time,
        )
        if phase == interval_steps:
            phase = 0


def main():
    from controller import Robot
    bridge = None
    live = None
    try:
        bridge = PPOBridge(MODEL_DIR)
        print('[PPO LOADED] ĐÃ LOAD THÀNH CÔNG PPO MODEL, VECNORM & KẾT NỐI NS-3.47!', flush=True)
        robot = Robot()
        if os.environ.get('LIVE_DASHBOARD', '1') != '0':
            try:
                live = LiveDashboard(port=int(os.environ.get('LIVE_PORT', '8765'))).start()
            except (OSError, ValueError) as exc:
                print(f'[LIVE WARNING] Không mở được dashboard: {exc}. Drone vẫn tiếp tục chạy.', flush=True)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('127.0.0.1', 8081))
            sock.setblocking(False)
            run(robot, bridge, sock, live=live)
    except Exception as exc:
        print(f'[CONTROLLER ERROR] {type(exc).__name__}: {exc}', flush=True)
        raise
    finally:
        if live is not None:
            live.close()
        if bridge is not None:
            bridge.close()


if __name__ == '__main__':
    main()
