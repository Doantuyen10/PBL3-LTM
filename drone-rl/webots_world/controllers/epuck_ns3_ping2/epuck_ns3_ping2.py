import socket
import json
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_relay_address = ("127.0.0.1", 8081)  # Gửi tới cổng nhận của Drone relay

EPUCK_ID = "E_PUCK_CLIENT_3"
seq = 0
SEND_INTERVAL = 2.0
last_send_time = robot.getTime() - SEND_INTERVAL

while robot.step(timestep) != -1:
    # Gửi ngay lần đầu, sau đó mỗi 2 giây mô phỏng (khớp đồng hồ UAV).
    if robot.getTime() - last_send_time >= SEND_INTERVAL - 1e-9:
        last_send_time = robot.getTime()
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