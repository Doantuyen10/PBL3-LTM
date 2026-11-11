# Kế hoạch phát triển & Lộ trình kỹ thuật — PBL3-LTM

Tài liệu này mô tả lộ trình thực hiện, hiện trạng kỹ thuật và các giai đoạn nâng cấp tiếp theo của dự án **Điều hướng và Lập quỹ đạo UAV nhận thức truyền thông (Communication-Aware UAV Trajectory Planning)**.

---

## 1. Mục tiêu tổng thể của dự án

Xây dựng hệ thống đồng mô phỏng (**Co-simulation Webots + PPO + ns-3**) trong đó UAV đóng vai trò trạm chuyển tiếp trên không (**UAV Relay / UAV-BS**), sử dụng thuật toán Học tăng cường sâu (**PPO**) để lập quỹ đạo bay sao cho:
1. Duy trì kết nối liên tục cho các nút mặt đất (`E_PUCK_CLIENT_1..3`) và trạm đích (`E_PUCK_SERVER`) trong phạm vi phủ sóng.
2. Tối đa hóa hiệu suất truyền thông (dung lượng đường truyền, tỷ lệ chuyển phát gói tin — PDR) và tối thiểu hóa độ trễ / rớt gói.
3. Đảm bảo ràng buộc động học và tiết kiệm năng lượng pin của UAV trong suốt nhiệm vụ.

---

## 2. Tổng kết các giai đoạn đã hoàn thành (Giai đoạn 1 – 3)

### Giai đoạn 1: Xây dựng môi trường cơ sở & Mô phỏng Webots — *Hoàn thành*
- [x] Thiết lập thế giới Webots ([VD2_vscode.wbt](drone-rl/webots_world/worlds/VD2_vscode.wbt) và [VD2.wbt](drone-rl/webots_world/worlds/VD2.wbt)) gồm 1 UAV `Mavic2Pro`, 3 nút phát `E_PUCK_CLIENT_1..3` và 1 nút thu `E_PUCK_SERVER`.
- [x] Xây dựng môi trường Gymnasium cơ sở [`UAVBSEnv`](drone-rl/webots_world/controllers/dieukhien/uav_bs_env.py) với mô hình kênh truyền Air-to-Ground (LoS/NLoS của Al-Hourani) và mô hình công suất đẩy cánh quạt (Zeng et al., 2019).
- [x] Thiết lập giao tiếp gói tin UDP giữa các robot mặt đất, UAV relay (cổng `8081`), e-puck server (cổng `9000`) và trình ghi log `ns-3.47` (cổng `8080`).

### Giai đoạn 2: Chuẩn hóa hợp đồng PPO & Huấn luyện mô hình Relay 3 Client — *Hoàn thành*
- [x] Định nghĩa môi trường chuyên biệt [`RelayEnv`](drone-rl/webots_world/controllers/dieukhien/relay_env.py) (`contract = "relay_3epuck_v1"`) ở độ cao `30 m`, bán kính phủ sóng `150 m`, chu kỳ `1.0 s`, giới hạn vận tốc `5 m/s` và gia tốc `2 m/s²`.
- [x] Huấn luyện mới hoàn toàn mô hình PPO (`ppo_relay_3epuck.zip` + `vecnorm_relay_3epuck.pkl`, 327.680 bước) với hàm thưởng ưu tiên liên kết xa nhất (worst-link distance) và phạt vượt ngưỡng `150 m`.
- [x] Xây dựng [`PPOBridge`](drone-rl/webots_world/controllers/dieukhien/ppo_bridge.py) kiểm tra mã băm SHA-256 của model, normalizer và môi trường trước khi chạy ([relay_model.json](drone-rl/webots_world/controllers/dieukhien/relay_model.json)).
- [x] Đồng bộ hóa đồng hồ mô phỏng (`robot.getTime()`) cho cả 3 client và nội suy quỹ đạo 125 bước (`8 ms`/bước) trong [dieukhien.py](drone-rl/webots_world/controllers/dieukhien/dieukhien.py).

### Giai đoạn 3: Kiểm thử tự động & Báo cáo thực nghiệm — *Hoàn thành*
- [x] Xây dựng bộ 12 unit tests trong [tests/test_ppo_controller.py](tests/test_ppo_controller.py) và công cụ chẩn đoán độ phủ sóng [scripts/check-ppo-coverage.py](scripts/check-ppo-coverage.py).
- [x] Đạt **100% độ phủ đồng thời cả 3 client và server** trong suốt 200 giây trên cấu hình world chuẩn; đạt trung bình **99,15%** trên 64 kịch bản nhiễu vị trí (100% các lượt chạy hội tụ hoàn toàn ở 50 giây cuối).
- [x] Hoàn thiện bộ báo cáo cuối kỳ ([bao_cao_cuoi_ky/](bao_cao_cuoi_ky/)) và bài báo nghiên cứu ([bao_cao_nghien_cuu/](bao_cao_nghien_cuu/)).

---

## 3. Kế hoạch phát triển tiếp theo (Giai đoạn 4 – 6)

Hiện tại, hệ thống còn **2 giới hạn kỹ thuật chính** đã được ghi nhận rõ trong code và báo cáo:
1. **Chưa đóng vòng phản hồi mạng từ ns-3 về PPO:** Chất lượng kênh (4 chiều cuối của vector quan sát 18 chiều) đang được tính bằng công thức giải tích A2G trong `uav_bs_env.py`, trong khi [epuck-ns3-server.cc](tools/ns-3.47/scratch/epuck-ns3-server.cc) mới chỉ nhận và in log gói tin một chiều.
2. **Điều khiển động học qua Supervisor:** Vị trí UAV đang được cập nhật trực tiếp qua `translation.setSFVec3f()` kèm `resetPhysics()`, chưa đi qua bộ điều khiển lực đẩy cánh quạt PID (như trong [mavic2pro_patrol.py](drone-rl/webots_world/controllers/mavic2pro_patrol/mavic2pro_patrol.py)).

Dưới đây là kế hoạch chi tiết để giải quyết triệt để các giới hạn này:

---

### Giai đoạn 4: Đóng vòng lặp đồng mô phỏng hai chiều Webots $\leftrightarrow$ ns-3 (Closed-Loop Network Feedback)

**Mục tiêu:** Thay thế việc cắt ngưỡng khoảng cách cứng (`150 m`) và công thức SNR giải tích tĩnh bằng mô phỏng mạng không dây thực sự trong `ns-3.47` (Wi-Fi 802.11a/g/n hoặc PropagationLossModel của ns-3).

#### Các hạng mục công việc:
1. **Nâng cấp `tools/ns-3.47/scratch/epuck-ns3-server.cc`:**
   - Khởi tạo `NodeContainer` gồm 5 nút trong ns-3: 3 nút Client, 1 nút UAV Relay (`MobilityModel` cập nhật tọa độ thực từ Webots), và 1 nút Server.
   - Cấu hình kênh truyền `YansWifiChannelHelper` + `FriisPropagationLossModel` / `LogDistancePropagationLossModel` (hoặc mô hình A2G).
   - Khi nhận tọa độ $(x, y, z)$ từ Webots mỗi bước: cập nhật `ConstantPositionMobilityModel` của UAV và các robot trong ns-3.
   - Bơm gói tin thực tế qua stack mạng ns-3 từ Client $\rightarrow$ UAV $\rightarrow$ Server và đo đạc:
     - **RSSI / SNR thực tế** trên từng liên kết.
     - **Throughput / Spectral Efficiency** thực đo.
     - **Packet Delivery Ratio (PDR)** và **End-to-End Delay**.
   - Gửi bản tin JSON phản hồi từ ns-3 về UAV controller qua cổng UDP (ví dụ `127.0.0.1:8082`).
2. **Cập nhật `ppo_bridge.py` & `dieukhien.py`:**
   - Bổ sung chế độ nhận 4 giá trị hiệu suất phổ / chất lượng kênh thực tế từ socket phản hồi của ns-3 để đưa vào `obs[14:18]` (với cơ chế fallback an toàn về công thức giải tích khi chạy offline test).
3. **Tiêu chí nghiệm thu (DoD):**
   - Khi UAV bay xa nút client, ns-3 tự động làm suy hao tín hiệu/rớt gói và phản hồi chỉ số suy giảm về `dieukhien.py`.
   - Log đầu cuối ghi nhận đầy đủ PDR, độ trễ trung bình (ms) và thông lượng (kbps) từ ns-3.

---

### Giai đoạn 5: Tích hợp bộ điều khiển vật lý cấp thấp (PID Flight Controller)

**Mục tiêu:** Chuyển từ điều khiển dịch chuyển động học (`Supervisor.setSFVec3f`) sang điều khiển vận tốc góc 4 động cơ cánh quạt thực tế của `Mavic2Pro`.

#### Các hạng mục công việc:
1. **Tách kiến trúc điều khiển 2 tầng (Hierarchical Control):**
   - **Tầng cao (High-level Planner — PPO, `1 Hz`):** Xuất tọa độ mục tiêu `target_xy` hoặc vận tốc tham chiếu `vel_ref` mỗi `1.0 s`.
   - **Tầng thấp (Low-level Tracker — PID, `125 Hz` / mỗi `8 ms`):** Tái sử dụng bộ điều khiển PID `roll / pitch / yaw / vertical thrust` từ [mavic2pro_patrol.py](drone-rl/webots_world/controllers/mavic2pro_patrol/mavic2pro_patrol.py) để điều khiển 4 propeller bám theo điểm đích `target` mà không cần gọi `resetPhysics()`.
2. **Tùy chọn cấu hình trong `dieukhien.py`:**
   - Hỗ trợ cờ `CONTROL_MODE = "kinematic" | "pid"` để vừa giữ tương thích tuyệt đối với các bài kiểm thử hình học hiện có, vừa chạy được mô phỏng động lực học cánh quạt thực tế.
3. **Tiêu chí nghiệm thu (DoD):**
   - UAV tự cất cánh bằng lực đẩy cánh quạt, giữ độ cao ổn định tại `30 m ± 0.5 m` và bám quỹ đạo PPO mà không bị lật hoặc vọt lố quá `2 m`.

---

### Giai đoạn 6: Mở rộng kịch bản động (Mobile Clients & Multi-UAV)

**Mục tiêu:** Đánh giá khả năng thích nghi của thuật toán DRL khi các nút mặt đất di chuyển và khi vùng cản trở tín hiệu thay đổi.

#### Các hạng mục công việc:
1. **Kịch bản E-puck di động (Mobile Ground Nodes):**
   - Lập trình cho 3 robot `E_PUCK_CLIENT_1..3` di chuyển theo quỹ đạo tuần tra hoặc ngẫu nhiên có hướng (Random Waypoint) với vận tốc `0.1 – 0.5 m/s`.
   - Huấn luyện bổ sung (fine-tune) chính sách PPO trên `RelayEnv` với `users` di chuyển theo thời gian.
2. **So sánh đa thuật toán (Baselines & Ablation Study):**
   - Bổ sung các baseline chuẩn để so sánh định lượng với PPO:
     - *Static Hover* (đứng yên tại tâm hình học / tâm Fermat-Weber).
     - *Circular Patrol* (bay tuần tra tròn).
     - *SAC / DDPG / TD3* trên cùng môi trường `RelayEnv`.
3. **Tiêu chí nghiệm thu (DoD):**
   - Bảng so sánh thống kê (trung bình $\pm$ độ lệch chuẩn trên $\ge 30$ seed) về: Tỷ lệ phủ sóng 3 client (%), PDR thực tế qua ns-3 (%), Năng lượng tiêu thụ (J), và Thời gian suy luận (ms/step).

---

## 4. Quy trình kiểm tra & Đảm bảo chất lượng (QA Checklist)

Mọi thay đổi trong các giai đoạn tiếp theo phải tuân thủ quy trình kiểm tra sau trước khi cập nhật báo cáo:

1. **Kiểm tra toàn vẹn mô hình & Unit Tests:**
   ```bash
   .venv/bin/python -m unittest discover -s tests -v
   ```
2. **Kiểm tra độ phủ sóng hình học 200 giây:**
   ```bash
   .venv/bin/python scripts/check-ppo-coverage.py
   ```
3. **Kiểm tra đồng bộ số liệu báo cáo (nếu cập nhật kết quả thực nghiệm):**
   ```bash
   .venv/bin/python bao_cao_cuoi_ky/nguon/danh_gia.py
   python3 bao_cao_cuoi_ky/nguon/ve_hinh.py
   python3 bao_cao_cuoi_ky/nguon/xuat_bao_cao.py
   ```
