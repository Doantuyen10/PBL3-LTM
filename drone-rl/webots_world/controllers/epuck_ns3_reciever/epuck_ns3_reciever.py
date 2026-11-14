import socket
import json
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# e-puck 0 làm Server lắng nghe tại cổng 9000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 9000))
sock.setblocking(False)

print("[E-PUCK 0 - SERVER] Đã sẵn sàng nhận gói tin từ mạng (qua xác thực ns-3 + UAV Relay)...")

while robot.step(timestep) != -1:
    try:
        data, addr = sock.recvfrom(2048)
        packet = json.loads(data.decode('utf-8'))

        sender = packet.get('sender')
        relay = packet.get('via_relay', 'DIRECT (Không qua Drone)')
        seq = packet.get('seq')
        distance = packet.get('distance_to_drone', 0.0)
        srv_dist = packet.get('distance_to_server', 0.0)
        delay_ms = packet.get('ns3_e2e_delay_ms')
        pdr = packet.get('ns3_pdr')

        if relay == "Drone_PPO":
            ns3_info = ""
            if delay_ms is not None and pdr is not None:
                ns3_info = f" | ns-3 delay={delay_ms}ms, PDR={100*float(pdr):.1f}%, d_srv={srv_dist}m"
            print(f"[E-PUCK 0 SERVER NHẬN] Gói tin #{seq} từ [{sender}] ---> "
                  f"ĐÃ ĐI QUA TRẠM TRUNG GIAN DRONE (d_up={distance}m{ns3_info})!")
        else:
            print(f"[E-PUCK 0 SERVER NHẬN] Gói tin #{seq} từ [{sender}] ---> NHẬN TRỰC TIẾP (Không qua Drone)!")

    except BlockingIOError:
        pass
