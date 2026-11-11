"""Webots UAV relay with a PPO UAV-BS observation/action adapter."""
from pathlib import Path
import json
import socket
import numpy as np
from ppo_bridge import PPOBridge, USER_DEFS, POLICY_DT

MODEL_DIR = Path(__file__).resolve().parent
CLIMB_SPEED = 5.0  # m/s, scripted takeoff; PPO starts only at the configured altitude.
COMM_RANGE_DRONE = 150.0


def find_users(robot):
    nodes = {name: robot.getFromDef(name) for name in USER_DEFS}
    missing = [name for name, node in nodes.items() if node is None]
    if missing:
        raise RuntimeError('World thiếu DEF: ' + ', '.join(missing)
                           + '. Mở VD2_vscode.wbt hoặc VD2.wbt đã cập nhật.')
    return nodes


def relay_packets(sock, position, nodes, received_at=None, sim_time=0.0):
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
        if distance > COMM_RANGE_DRONE:
            print(f'[UDP DROP] {sender} seq={packet.get("seq")} d={distance:.2f} m', flush=True)
            continue
        packet.update(via_relay='Drone_PPO', relay_node='Drone_PPO',
                      distance_to_drone=round(distance, 2))
        payload = json.dumps(packet).encode('utf-8')
        for port in (9000, 8080):
            sock.sendto(payload, ('127.0.0.1', port))
        print(f'[UDP RELAY] {sender} seq={packet.get("seq")} d={distance:.2f} m', flush=True)


def log_links(position, nodes, received_at, sim_time):
    """Report geometry separately from reception; no end-to-end ACK exists."""
    for sender in USER_DEFS[:3]:
        distance = float(np.linalg.norm(position - np.asarray(nodes[sender].getPosition())))
        coverage = 'IN_RANGE' if distance <= COMM_RANGE_DRONE else 'OUT_OF_RANGE'
        last = received_at.get(sender)
        reception = ('NO_PACKETS_YET' if last is None else
                     'RX_OK' if sim_time - last <= 6.0 else 'RX_STALE')
        print(f'[LINK] {sender} d={distance:.2f}m {coverage} {reception}', flush=True)


def run(robot, bridge, sock):
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0
    interval_steps = round(POLICY_DT / dt)
    if interval_steps < 1 or not np.isclose(interval_steps * dt, POLICY_DT):
        raise ValueError('basicTimeStep phải chia hết chu kỳ PPO 1000 ms (world dùng 8 ms).')
    uav = robot.getSelf()
    if uav is None:
        raise RuntimeError('Không lấy được UAV; cần supervisor TRUE.')
    translation = uav.getField('translation')
    nodes = find_users(robot)
    users = lambda: [nodes[name].getPosition() for name in USER_DEFS]
    motors = []
    for name, sign in [('front left propeller', 1), ('front right propeller', -1),
                       ('rear left propeller', -1), ('rear right propeller', 1)]:
        motor = robot.getDevice(name)
        if motor is not None:
            motor.setPosition(float('inf'))
            motor.setVelocity(sign * 10.0)
            motors.append(motor)
    print(f'[PPO READY] Model={MODEL_DIR / "ppo_relay_3epuck.zip"}; obs=18; action=2; '
          f'chu kỳ={POLICY_DT:g}s; CPU', flush=True)
    print('[PPO CONFIG] 4 node = 3 client + server; xy mô hình = xy Webots + 500 m. '
          'Môi trường relay_3epuck_v1, ưu tiên cả 3 client; chưa có phản hồi ns-3.', flush=True)
    active = False
    phase = 0
    start = target = None
    last_climb_log = -1
    ended_logged = False
    received_at = {}
    sim_time = 0.0
    next_link_log = 0.0
    while robot.step(timestep) != -1:
        pos = np.asarray(translation.getSFVec3f(), dtype=float)
        sim_time += dt
        relay_packets(sock, pos, nodes, received_at, sim_time)
        if sim_time >= next_link_log:
            log_links(pos, nodes, received_at, sim_time)
            next_link_log = sim_time + 5.0
        if not active:
            delta = bridge.altitude - pos[2]
            pos[2] += np.clip(delta, -CLIMB_SPEED * dt, CLIMB_SPEED * dt)
            translation.setSFVec3f(pos.tolist())
            uav.resetPhysics()
            height_band = int(pos[2] // 10)
            if height_band != last_climb_log:
                print(f'[TAKEOFF] z={pos[2]:.1f}/{bridge.altitude:g} m', flush=True)
                last_climb_log = height_band
            if abs(pos[2] - bridge.altitude) <= 1e-6:
                bridge.start(pos, users())
                active = True
                print('[PPO ACTIVE] PPO tiếp quản chuyển động ngang.', flush=True)
            continue
        if phase == 0:
            if bridge.done:
                # Hold final target; do not silently reset energy or teleport.
                translation.setSFVec3f(target.tolist())
                uav.resetPhysics()
                if not ended_logged:
                    print('[PPO DONE] Hết episode/pin mô hình. Giữ vị trí; reset world để chạy lại.', flush=True)
                    ended_logged = True
                continue
            start = pos.copy()
            target, action, info = bridge.next_target(pos, users())
            print(f'[PPO STEP {bridge.env.t:03d}] action={action.round(4).tolist()} '
                  f'velocity={bridge.env.vel.round(3).tolist()} '
                  f'target={target.round(3).tolist()} battery={bridge.env.E:.1f} J', flush=True)
        phase += 1
        # Interpolate the 1 s model transition over 125 Webots steps (8 ms).
        new_pos = start + (target - start) * (phase / interval_steps)
        translation.setSFVec3f(new_pos.tolist())
        uav.resetPhysics()
        if phase == interval_steps:
            phase = 0


def main():
    from controller import Supervisor
    bridge = None
    try:
        bridge = PPOBridge(MODEL_DIR)
        print('[PPO LOADED] ĐÃ LOAD THÀNH CÔNG PPO MODEL & VECNORM!', flush=True)
        robot = Supervisor()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('127.0.0.1', 8081))
            sock.setblocking(False)
            run(robot, bridge, sock)
    except Exception as exc:
        print(f'[CONTROLLER ERROR] {type(exc).__name__}: {exc}', flush=True)
        raise
    finally:
        if bridge is not None:
            bridge.close()


if __name__ == '__main__':
    main()
