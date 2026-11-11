# Cài đặt trên máy mới

Xem [INSTALL.md](INSTALL.md). Sao chép `.env.example` thành `.env`, chỉnh đường dẫn Webots rồi dùng `scripts/run_project.py` để chạy.

# PBL3-LTM: Điều hướng và Lập quỹ đạo UAV nhận thức truyền thông (Communication-Aware UAV Trajectory Planning)

Dự án nghiên cứu và triển khai thuật toán Học tăng cường sâu (**Deep Reinforcement Learning — PPO**) để điều khiển quỹ đạo bay của UAV đóng vai trò trạm chuyển tiếp trên không (**UAV Relay / Aerial Base Station**), kết hợp mô phỏng vật lý trong **Webots** và ghi nhận/kiểm chứng gói tin mạng qua **UDP / ns-3**.

---

## 1. Giới thiệu bài toán

Trong các kịch bản truyền thông mặt đất bị giới hạn tầm phủ hoặc cản trở tín hiệu, một UAV (`Mavic2Pro`) được triển khai ở độ cao cố định để làm trạm chuyển tiếp dữ liệu giữa các nút cảm biến/robot mặt đất (`E-puck`) và trạm thu trung tâm (`Server`).

- **Mục tiêu điều khiển:** Điều hướng UAV theo thời gian thực để duy trì đồng thời **3 nút client** (`E_PUCK_CLIENT_1`, `E_PUCK_CLIENT_2`, `E_PUCK_CLIENT_3`) và **1 nút server** (`E_PUCK_SERVER`) nằm trong bán kính truyền thông `150 m`, đồng thời tối ưu hóa chất lượng liên kết yếu nhất (max-min link distance) và tiết kiệm năng lượng bay.
- **Cơ chế chuyển tiếp gói tin (UDP Relay):**
  - Ba robot client phát gói tin định kỳ mỗi `2.0 s` (theo đồng hồ mô phỏng Webots) tới cổng UDP `127.0.0.1:8081` của UAV.
  - Controller trên UAV kiểm tra khoảng cách hình học 3D thực tế giữa UAV và nguồn phát trong Webots:
    - Nếu khoảng cách $\le 150\text{ m}$: Đóng dấu xác thực chuyển tiếp (`via_relay="Drone_PPO"`) kèm khoảng cách thực tế, sau đó chuyển tiếp đồng thời tới **Server e-puck** (`127.0.0.1:9000`) và **ns-3 Packet Log Server** (`127.0.0.1:8080`).
    - Nếu khoảng cách $> 150\text{ m}$: Đánh rớt gói tin (`[UDP DROP]`).

---

## 2. Kiến trúc hệ thống

```text
+-----------------------+       UDP :8081        +----------------------------------+
|  3x E-puck Clients    | ---------------------> |       UAV Relay (Mavic2Pro)      |
|  - E_PUCK_CLIENT_1    |   JSON Data Packets    |  - Controller: dieukhien.py      |
|  - E_PUCK_CLIENT_2    |   (chu kỳ 2.0s sim)    |  - Adapter: ppo_bridge.py        |
|  - E_PUCK_CLIENT_3    |                        |  - Policy: ppo_relay_3epuck.zip  |
+-----------------------+                        +----------------------------------+
                                                          |                  |
                                   Kiểm tra d_3D <= 150m  |                  |
                                      +-------------------+                  +-------------------+
                                      | UDP :9000                                                | UDP :8080
                                      v                                                          v
                        +---------------------------+                              +---------------------------+
                        |      E-puck Server        |                              |   ns-3.47 Packet Server   |
                        |  - E_PUCK_SERVER          |                              |  - epuck-ns3-server.cc    |
                        |  - epuck_ns3_reciever.py  |                              |  - Xác thực & ghi log gói |
                        +---------------------------+                              +---------------------------+
```

---

## 3. Cấu trúc thư mục

```text
PBL3-LTM/
├── drone-rl/
│   └── webots_world/
│       ├── worlds/
│       │   ├── VD2_vscode.wbt             # World chính dùng controller <extern> (chạy/debug qua VS Code F5)
│       │   ├── VD2.wbt                    # World dùng controller nội bộ "dieukhien"
│       │   └── mavic_2_pro.wbt            # World mẫu mặc định của Webots
│       └── controllers/
│           ├── dieukhien/                 # Controller UAV PPO & môi trường RL
│           │   ├── dieukhien.py           # Vòng lặp điều khiển Webots + chuyển tiếp gói UDP
│           │   ├── ppo_bridge.py          # Adapter nạp PPO, VecNormalize, kiểm tra SHA-256 & ánh xạ tọa độ
│           │   ├── relay_env.py           # Môi trường huấn luyện RelayEnv (contract: relay_3epuck_v1)
│           │   ├── uav_bs_env.py          # Môi trường cơ sở UAVBSEnv (kênh A2G LoS/NLoS & công suất bay)
│           │   ├── ppo_relay_3epuck.zip   # Trọng số mô hình PPO đã huấn luyện (327.680 bước)
│           │   ├── vecnorm_relay_3epuck.pkl # Bộ chuẩn hóa quan sát VecNormalize đi kèm
│           │   └── relay_model.json       # Metadata cấu hình & mã băm SHA-256 kiểm tra toàn vẹn
│           ├── epuck_ns3_ping/            # Controller phát gói UDP của E_PUCK_CLIENT_1
│           ├── epuck_ns3_ping1/           # Controller phát gói UDP của E_PUCK_CLIENT_2
│           ├── epuck_ns3_ping2/           # Controller phát gói UDP của E_PUCK_CLIENT_3
│           └── epuck_ns3_reciever/        # Controller nhận gói UDP của E_PUCK_SERVER (cổng 9000)
├── scripts/
│   ├── setup-ns3.sh                       # Script tự động tải và biên dịch cục bộ ns-3.47
│   └── check-ppo-coverage.py              # Script kiểm tra hình học độ phủ sóng 200s (nội suy 8ms)
├── tests/
│   └── test_ppo_controller.py             # Bộ 12 unit tests kiểm thử toàn diện controller, PPO và UDP relay
├── tools/
│   └── ns-3.47/
│       └── scratch/epuck-ns3-server.cc    # Chương trình ns-3 lắng nghe và xác thực gói tin tại cổng 8080
├── bao_cao_cuoi_ky/                       # Báo cáo cuối kỳ (PDF/DOCX/MD), biểu đồ và script tái lập thực nghiệm
├── bao_cao_nghien_cuu/                    # Bài báo nghiên cứu mô hình PPO Relay 3 e-puck kèm minh chứng
├── Tailieu/                               # Đề tài và các bài báo khoa học tham khảo
└── VSCODE_WEBOTS.md                       # Hướng dẫn vận hành chi tiết trên VS Code & chẩn đoán lỗi
```

---

## 4. Đặc tả mô hình PPO (`relay_3epuck_v1`)

Mô hình điều khiển UAV sử dụng thuật toán **PPO** (`stable-baselines3`) kết hợp chuẩn hóa quan sát **`VecNormalize`**:

| Thông số | Giá trị / Mô tả |
| :--- | :--- |
| **Độ cao hoạt động ($H$)** | `30.0 m` (cất cánh tự động với vận tốc `5.0 m/s` trước khi bật PPO) |
| **Bán kính phủ sóng ($R_{\text{comm}}$)** | `150.0 m` (tính theo khoảng cách Euclid 3D) |
| **Chu kỳ quyết định ($\Delta t$)** | `1.0 s` (nội suy tuyến tính qua `125` bước mô phỏng Webots `8 ms`) |
| **Thời lượng 1 Episode ($T$)** | `200 bước` (`200 s` mô phỏng), dung lượng pin khởi tạo `40.000 J` |
| **Giới hạn động học** | Vận tốc ngang tối đa $v_{\max} = 5.0\text{ m/s}$, gia tốc tối đa $a_{\max} = 2.0\text{ m/s}^2$ |
| **Ánh xạ hệ tọa độ** | $(x_{\text{model}}, y_{\text{model}}) = (x_{\text{webots}} + 500\text{ m}, y_{\text{webots}} + 500\text{ m})$ |
| **Không gian quan sát (`18` chiều)** | `pos_xy` (2) + `vel_xy` (2) + `battery` (1) + `time_left` (1) + `rel_xy_4_nodes` (8) + `spectral_eff_4_nodes` (4) |
| **Không gian hành động (`2` chiều)** | Gia tốc ngang chuẩn hóa $(a_x, a_y) \in [-1, 1]^2$ |

- **Tọa độ các nút trong Webots (`VD2_vscode.wbt` / `VD2.wbt`):**
  - `E_PUCK_CLIENT_1`: `(-100, -50, 0)`
  - `E_PUCK_CLIENT_2`: `(50, -50, 0)`
  - `E_PUCK_CLIENT_3`: `(100, 100, 0)`
  - `E_PUCK_SERVER`: `(2, 0, 0)`

---

## 5. Hướng dẫn cài đặt và vận hành

### 5.1. Yêu cầu môi trường
- **Hệ điều hành:** Linux (Ubuntu/Debian khuyến nghị)
- **Webots:** Cài đặt tại `/usr/local/webots`
- **Python:** Môi trường ảo `.venv` tại thư mục gốc dự án có cài đặt `numpy`, `gymnasium`, `stable-baselines3`, `torch`.
- **ns-3 (tùy chọn khi chạy server log C++):** Cài đặt cục bộ qua script:
  ```bash
  bash scripts/setup-ns3.sh
  ```

### 5.2. Chạy mô phỏng Webots + PPO bằng VS Code (Khuyến nghị)
1. Mở thư mục `/home/doantuyen/PBL3-LTM` trong VS Code.
2. Mở Command Palette (`Ctrl+Shift+P`) $\rightarrow$ **Tasks: Run Task** $\rightarrow$ chọn **`Webots: Open Mavic world`** để mở `VD2_vscode.wbt`.
3. Trong cửa sổ Webots, nhấn nút **Run** (UAV ở chế độ chờ controller `<extern>`).
4. Trong VS Code, mở **Run and Debug** $\rightarrow$ chọn cấu hình **`Webots: Mavic PPO`** $\rightarrow$ nhấn **F5**.

### 5.3. Chạy controller ngoài trực tiếp từ Terminal
Sau khi mở `VD2_vscode.wbt` trong Webots và nhấn **Run**, chạy lệnh sau tại thư mục gốc dự án:

```bash
WEBOTS_HOME=/usr/local/webots \
PYTHONPATH="/usr/local/webots/lib/controller/python${PYTHONPATH:+:$PYTHONPATH}" \
LD_LIBRARY_PATH="/usr/local/webots/lib/controller${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
.venv/bin/python -u drone-rl/webots_world/controllers/dieukhien/dieukhien.py
```

### 5.4. Chạy kèm bộ ghi log gói tin ns-3 (Cổng 8080)
Trong một terminal khác, khởi chạy chương trình lắng nghe gói tin của `ns-3.47`:

```bash
cd tools/ns-3.47
./ns3 run scratch/epuck-ns3-server.cc
```

> Xem thêm hướng dẫn chẩn đoán kết nối (`[LINK]`, `IN_RANGE`, `RX_OK`, `RX_STALE`) trong tệp [VSCODE_WEBOTS.md](VSCODE_WEBOTS.md).

---

## 6. Kiểm thử & Đánh giá độ phủ sóng

### 6.1. Chạy bộ kiểm thử tự động (Unit Tests)
Dự án đi kèm 12 bài kiểm thử tự động kiểm tra hợp đồng quan sát 18 chiều, giới hạn gia tốc/vận tốc/biên, chu kỳ điều khiển 1 giây (125 bước nội suy), khả năng lọc gói UDP lỗi và đảm bảo quỹ đạo PPO giữ đủ 3 client trong bán kính 150 m:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

### 6.2. Chẩn đoán hình học độ phủ sóng (Offline Rollout)
Kiểm tra nhanh quỹ đạo 200 giây của checkpoint PPO trên tọa độ thực của world (nội suy từng bước 8 ms):

```bash
.venv/bin/python scripts/check-ppo-coverage.py
```

### 6.3. Tóm tắt kết quả đánh giá mô hình (`ppo_relay_3epuck.zip`)
- **Trên cấu hình world chuẩn (200 giây):** Tỷ lệ duy trì đồng thời cả 3 client và server trong bán kính `150 m` đạt **100%** (khoảng cách client xa nhất cực đại là `144,56 m`, năng lượng tiêu thụ `33.105 J`).
- **Trên 64 kịch bản kiểm thử ngẫu nhiên (held-out seeds có nhiễu vị trí):** Thời gian phủ sóng đồng thời cả 3 client đạt trung bình **99,15%**, và **64/64 lượt chạy** duy trì 100% kết nối trong 50 giây cuối của episode.

---

## 7. Báo cáo & Tài liệu tái lập

- **Báo cáo cuối kỳ (`bao_cao_cuoi_ky/`):**
  - Tài liệu hoàn chỉnh: `Bao_cao_cuoi_ky_PBL3_LTM.pdf` (và `.docx`, `.md`).
  - Xem hướng dẫn tái lập số liệu và biểu đồ tại [bao_cao_cuoi_ky/README.md](bao_cao_cuoi_ky/README.md).
- **Bài báo nghiên cứu (`bao_cao_nghien_cuu/`):**
  - Tài liệu: `Bai_bao_PPO_Relay_3_e_puck.pdf` (và `.docx`, `.md`) cùng dữ liệu đánh giá tại `bao_cao_nghien_cuu/minh_chung/evaluation.json`.
