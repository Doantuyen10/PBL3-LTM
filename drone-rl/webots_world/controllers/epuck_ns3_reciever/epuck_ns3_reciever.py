import socket
import json
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# e-puck 0 làm Server lắng nghe tại cổng 9000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 9000))
sock.setblocking(False)

print("[E-PUCK 0 - SERVER] Đã sẵn sàng nhận gói tin từ mạng...")

while robot.step(timestep) != -1:
    try:
        data, addr = sock.recvfrom(2048)
        packet = json.loads(data.decode('utf-8'))
        
        sender = packet.get('sender')
        relay = packet.get('via_relay', 'DIRECT (Không qua Drone)')
        seq = packet.get('seq')
        distance = packet.get('distance_to_drone', 0.0)
        
        if relay == "Drone_PPO":
            print(f"[E-PUCK 0 SERVER NHẬN] Gói tin #{seq} từ [{sender}] ---> ĐÃ ĐI QUA TRẠM TRUNG GIAN DRONE (Khoảng cách Drone: {distance}m)!")
        else:
            print(f"[E-PUCK 0 SERVER NHẬN] Gói tin #{seq} từ [{sender}] ---> NHẬN TRỰC TIẾP (Không qua Drone)!")
            
    except BlockingIOError:
        pass