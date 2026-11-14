import socket
import json
import time
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_relay_address = ("127.0.0.1", 8081)  # Gửi tới cổng nhận của Drone relay

EPUCK_ID = "E_PUCK_CLIENT_1"
seq = 0
last_send_time = time.time()

while robot.step(timestep) != -1:
    # Gửi định kỳ mỗi 2 giây
    if time.time() - last_send_time >= 2.0:
        last_send_time = time.time()
        seq += 1
        
        # Đóng gói dữ liệu gói tin mạng thực tế
        packet = {
            "type": "DATA_PACKET",
            "sender": EPUCK_ID,
            "destination": "SERVER_DEST",
            "seq": seq,
            "payload": f"Hello from {EPUCK_ID} via Drone Relay"
        }
        
        try:
            sock.sendto(json.dumps(packet).encode('utf-8'), drone_relay_address)
            print(f"[{EPUCK_ID}] Đã phát gói tin #{seq} hướng tới Drone.")
        except Exception as e:
            print(f"Lỗi gửi gói tin: {e}")