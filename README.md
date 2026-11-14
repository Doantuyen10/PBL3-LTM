# PBL3-LTM-NS3: Điều hướng UAV Relay bằng PPO có đóng vòng phản hồi mạng ns-3.47 (Closed-Loop Co-Simulation)

Thư mục dự án **`PBL3-LTM-NS3`** là phiên bản nâng cấp từ `PBL3-LTM`, giải quyết triệt để hạn chế đã nêu trong báo cáo cuối kỳ (*"Mô phỏng mạng ns-3 chưa tham gia vòng điều khiển; các chỉ số kênh được tính giải tích, chưa lấy từ ns-3; chưa lọc khoảng cách chặng UAV–server"*).

---

## 1. Điểm nâng cấp cốt lõi so với `PBL3-LTM`

| Hạn chế ở bản cũ (`PBL3-LTM`) | Giải pháp đã triển khai ở bản mới (`PBL3-LTM-NS3`) |
| :--- | :--- |
| `epuck-ns3-server.cc` chỉ nhận UDP ở cổng 8080 để in log một chiều. | Nâng cấp thành **Closed-Loop Co-Simulation Server** (`ns-3.47`) với 5 nút `ConstantPositionMobilityModel`, chuỗi kênh truyền `FriisPropagationLossModel` + `AirToGroundExcessLossModel` (LoS/NLoS + suy hao vật cản) và `ConstantSpeedPropagationDelayModel`. |
| Vector quan sát `obs[14:18]` (hiệu suất phổ 4 nút) chỉ tính bằng công thức giải tích tĩnh trong Python. | Ở mỗi bước điều khiển, `PPOBridge` truy vấn trực tiếp `ns-3.47` (`STATE_QUERY`) để lấy `spectral_eff`, `snr_db`, `rx_power_dbm` thực tế từ `ns-3` nạp vào `obs[14:18]`. |
| `ns-3` không tác động lên hành động điều khiển quỹ đạo của UAV. | Khi `ns-3` phát hiện suy giảm SNR (do khoảng cách hoặc vật cản che khuất `shadow_loss_db`), `ns-3` xuất vector gia tốc phục hồi `control_accel` và hệ số `assist_weight` kết hợp bộ điều khiển PD có biên bảo vệ (`142 m`) để kéo UAV về phía liên kết yếu nhất. |
| Chỉ kiểm tra khoảng cách chặng `Client -> UAV`, bỏ qua chặng `UAV -> Server`. | Kiểm tra **đủ cả 2 chặng (`Client -> UAV` và `UAV -> Server`)** cả về khoảng cách ($\le 150\text{ m}$) lẫn ngưỡng $\text{SNR} \ge 10,35\text{ dB}$ trong `ns-3.47`. |
| Chưa đo được PDR, Throughput và End-to-End Delay qua `ns-3`. | Tự động đo và phản hồi trực tiếp **PDR (%)**, **Throughput (bps)**, **End-to-End 2-hop Delay (ms)** trong từng bản tin `NS3_PACKET_VERDICT`. |

---

## 2. Kiến trúc vòng kín (Closed-Loop Architecture)

```text
+-----------------------+      UDP :8081      +---------------------------------------------+
|  3x E-puck Clients    | ------------------> |          UAV Relay (dieukhien.py)           |
|  E_PUCK_CLIENT_1..3   |   DATA_PACKET (2s)  |  - PPOBridge + Ns3CoSimClient               |
+-----------------------+                     |  - Quan sát obs[14:18] lấy từ ns-3.47       |
                                              |  - Hành động: (1-w)*a_PPO + w*a_NS3_PD      |
                                              +---------------------------------------------+
                                                     |                               ^
                        1. STATE_QUERY / DATA_PACKET | UDP :8082                     | 2. NS3_FEEDBACK / VERDICT
                           (uav_pos, users_pos)      v                               |    (spectral_eff, snr_db,
                                              +---------------------------------------------+  control_accel, pdr, delay)
                                              |    ns-3.47 Co-Simulation Server (C++)       |
                                              |  - epuck-ns3-server.cc                      |
                                              |  - 5x ConstantPositionMobilityModel         |
                                              |  - Friis + A2G LoS/NLoS + Shadowing Model   |
                                              |  - Kiểm tra 2 chặng: Uplink & Downlink      |
                                              +---------------------------------------------+
                                                     |
                        3. Nếu forward_allowed=true  | UDP :9000 (kèm ns3_e2e_delay_ms, ns3_pdr)
                                                     v
                                              +---------------------------------------------+
                                              |       E-puck Server (E_PUCK_SERVER)         |
                                              |  - epuck_ns3_reciever.py                    |
                                              +---------------------------------------------+
```

---

## 3. Kết quả thực nghiệm đóng vòng với `ns-3.47`

Chạy kiểm chứng bằng lệnh:
```bash
.venv/bin/python scripts/check-ppo-coverage.py
```

| Kịch bản (200 giây, 300 gói UDP) | Chế độ điều khiển | Độ phủ hình học 3 Client | SNR Client 3 cuối lượt | Số gói tới Server | PDR (%) | Độ trễ 2 chặng TB (ms) | Thông lượng (bps) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Không vật cản (`0 dB`)** | **Closed-Loop (`PPO + ns-3`)** | **100,0%** | `12,34 dB` (OK) | `300 / 300` | **100,00%** | `5,071 ms` | `6.123,15 bps` |
| **Vật cản `4.0 dB` tại Client 3** | **Open-Loop (Bản cũ, không có `ns-3`)** | 100,0% | `8,36 dB` (**Mất kết nối**) | `200 / 300` | **66,67%** | `4,848 ms` | `4.081,78 bps` |
| **Vật cản `4.0 dB` tại Client 3** | **Closed-Loop (`PPO + ns-3`)** | **100,0%** | **`10,54 dB` (Phục hồi OK)** | **`295 / 300`** | **98,33%** | `5,579 ms` | **`6.027,80 bps`** |

> **Nhận xét:** Khi có vật cản gây suy hao `4.0 dB` tại `E_PUCK_CLIENT_3`, bộ điều khiển cũ (chỉ dựa vào công thức giải tích tĩnh) không biết kênh bị suy hao nên đứng ở khoảng cách `~128 m`, làm toàn bộ 100 gói tin của Client 3 bị rớt (`PDR = 66,67%`). Khi **đóng vòng với `ns-3.47`**, `ns-3` báo suy giảm SNR về controller và điều hướng UAV tiến lại gần Client 3 (`~110 m`) đồng thời giữ Client 1 trong ngưỡng `150 m`, giúp khôi phục hoàn toàn liên kết và nâng PDR lên **98,33%** (`295/300` gói thành công, chỉ mất 5 gói đầu lúc UAV đang bay tới điểm cân bằng mới).

---

## 4. Hướng dẫn biên dịch và chạy

### 4.1. Biên dịch lại server `ns-3.47` (nếu sửa code C++)
```bash
./scripts/build-ns3-server.sh
```

### 4.2. Chạy bộ kiểm thử tự động (14 Unit & Integration Tests với binary `ns-3.47` thật)
```bash
.venv/bin/python -m unittest discover -s tests -v
```

### 4.3. Chạy mô phỏng Webots
1. **Cách 1 (Tự động khởi chạy `ns-3` ngầm):** Chỉ cần mở `VD2_vscode.wbt` trong Webots, nhấn **Run**, và nhấn **F5** (`Webots: Mavic PPO`) trong VS Code. `Ns3CoSimClient` sẽ tự động kiểm tra cổng `8082` và khởi chạy tiến trình `ns3.47-epuck-ns3-server-default` nếu chưa bật.
2. **Cách 2 (Xem trực tiếp log C++ của `ns-3` trên Terminal riêng):**
   - Terminal 1:
     ```bash
     ./tools/ns-3.47/build/scratch/ns3.47-epuck-ns3-server-default --port=8082
     ```
   - Terminal 2 (hoặc F5 trong VS Code):
     ```bash
     WEBOTS_HOME=/usr/local/webots \
     PYTHONPATH="/usr/local/webots/lib/controller/python${PYTHONPATH:+:$PYTHONPATH}" \
     LD_LIBRARY_PATH="/usr/local/webots/lib/controller${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
     .venv/bin/python -u drone-rl/webots_world/controllers/dieukhien/dieukhien.py
     ```


### 4.4. Drone chạy với `supervisor FALSE`
Cả `VD2.wbt` và `VD2_vscode.wbt` dùng `Robot`, GPS/IMU/gyro và bốn động cơ để bay vật lý. Controller không đọc scene tree, đặt trực tiếp translation hay resetPhysics. `CONTROL_MODE` mặc định là `physics`; `auto` cũng chạy vật lý, còn `kinematic` không được hỗ trợ.

Tọa độ bốn E-puck đứng yên được khai báo trong `customData.users_pos` của Mavic2Pro, theo hệ XYZ của Webots và đơn vị mét, phục vụ PPO, relay và ns-3. Khi đổi vị trí E-puck trong world, cần cập nhật mục tương ứng trong `customData`; cấu hình này không theo dõi E-puck di chuyển. Vị trí drone được đọc trực tiếp từ GPS.

Các số liệu thực nghiệm ở trên là kết quả kiểm tra quỹ đạo mô hình trước thay đổi này; cần đo lại trên Webots để đánh giá chuyến bay vật lý.

Nếu `VD2_vscode.wbt` dùng controller `dieukhien`, Webots tự chạy controller. Chỉ dùng F5 khi trường controller được đặt thành `<extern>`.

### 4.5. Dashboard live
Khi controller `dieukhien` chạy, dashboard tự mở dịch vụ tại **http://127.0.0.1:8765**. Mở địa chỉ này bằng trình duyệt trên cùng máy chạy Webots. Nếu Webots đang mở từ trước khi cập nhật code, khởi động lại controller hoặc reload world.

Dashboard hiển thị:
- Vị trí GPS, quỹ đạo XY và điểm đích PPO; vùng phủ hình học 150 m.
- Độ cao, vận tốc ước lượng từ GPS, năng lượng mô hình PPO và biểu đồ độ cao.
- Khoảng cách tới 3 client và server, SNR/trạng thái từ phản hồi ns-3 gần nhất.
- Số gói nhận/chuyển/loại tại relay, PDR và thông lượng ns-3, độ trễ gói gần nhất, 30 sự kiện gói tin.
- Trạng thái chờ dữ liệu, dữ liệu cũ khi tạm dừng và tự kết nối lại khi controller khởi động lại.

`Xóa quỹ đạo hiển thị` chỉ xóa lịch sử trong trình duyệt. `Tải dữ liệu JSON` xuất snapshot và tối đa 1.200 mẫu đang lưu ở trình duyệt; lịch sử bắt đầu khi mở trang, không phải bản ghi toàn bộ chuyến bay. Thống kê relay tính từ khi controller khởi động; thống kê ns-3 có thể được reset khi PPO bắt đầu.

Dashboard lấy mẫu tối đa 10 lần/giây thực và trình duyệt cập nhật khoảng 4 lần/giây. Server HTTP chạy ở luồng riêng, chỉ đọc snapshot; không gọi API Webots hay gửi thêm truy vấn ns-3. Chỉ lắng nghe tại localhost, không cần cài thư viện web hay truy cập Internet.

Tùy chọn biến môi trường:
```bash
LIVE_PORT=8766        # Đổi cổng khi 8765 đang được sử dụng
LIVE_DASHBOARD=0      # Tắt dashboard
```
Đặt biến trong môi trường khởi chạy Webots hoặc mục `env` của cấu hình F5. Nếu mở cổng thất bại, controller báo `[LIVE WARNING]` và vẫn tiếp tục điều khiển drone.

Kiểm tra phần live không cần mở socket:
```bash
.venv/bin/python -m unittest discover -s tests -p test_live_dashboard.py -v
node tests/live_dashboard_ui.test.cjs
```
