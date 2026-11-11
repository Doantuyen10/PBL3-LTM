# Chạy UAV relay nhận điều khiển từ PPO

## Chạy bằng VS Code

1. Mở thư mục `/home/doantuyen/PBL3-LTM`.
2. **Tasks: Run Task → Webots: Open Mavic world** để mở `VD2_vscode.wbt`.
3. Trong Webots nhấn **Run**. UAV dùng controller `<extern>`.
4. **Run and Debug → Webots: Mavic PPO → F5** để chạy `dieukhien.py` bằng `.venv`.
5. Xem lần lượt các thông báo:
   - `[PPO LOADED]`: đã nạp checkpoint và VecNormalize.
   - `[PPO READY]`: đã kết nối Webots, tìm đủ bốn node và kiểm tra bước thời gian.
   - `[TAKEOFF]`: đang tăng độ cao; mặc định 5 m/s đến 30 m, khoảng 6 giây mô phỏng.
   - `[PPO ACTIVE]`: chính sách bắt đầu điều khiển ngang.
   - `[PPO STEP 001]`: hành động, vận tốc, tọa độ đích và pin mô hình. Cập nhật mỗi giây mô phỏng.
   - `[PPO DONE]`: hết 200 bước chính sách hoặc pin mô hình; UAV giữ vị trí. Reset world và khởi động lại controller để chạy episode mới.

`VD2.wbt` dùng controller nội bộ `dieukhien`, có cùng bốn node mặt đất. Để chạy world này, Webots cần dùng Python có các thư viện trong `.venv`. World `mavic_2_pro.wbt` cũ không có các node relay yêu cầu; dùng một trong hai world VD2 khi chạy controller PPO mới.

Chỉ mở một world dùng controller ngoài để tránh kết nối nhầm. Khi sửa controller hoặc chạy lại, dừng F5, reset/reload world rồi chạy lại. API `controller` lấy từ Webots, không cài gói PyPI cùng tên.

## Giao diện PPO đang áp dụng

`ppo_bridge.py` nạp `ppo_relay_3epuck.zip` và `vecnorm_relay_3epuck.pkl` từ cùng thư mục, kiểm tra không gian quan sát `(18,)` và hành động `(2,)`, dùng CPU và đóng băng VecNormalize. Thông báo lỗi nêu rõ đường dẫn tệp thiếu hoặc lỗi cấu hình; không âm thầm dùng hành động bằng 0 khi nạp thất bại.

`relay_env.py` là môi trường đã dùng để huấn luyện checkpoint relay mới trong
`/home/doantuyen/PPO/relay_3epuck`. Metadata và SHA-256 của model, normalizer,
môi trường nằm trong `relay_model.json`; controller kiểm tra chúng trước khi chạy.
Các tệp checkpoint cũ vẫn được giữ nguyên nhưng không còn là bộ đang chạy.

- Ba client theo thứ tự `E_PUCK_CLIENT_1`, `E_PUCK_CLIENT_2`, `E_PUCK_CLIENT_3`;
  server `E_PUCK_SERVER` là nút thứ tư trong quan sát. Reward xét cả liên kết server.
- Tọa độ thật qua DEF/Supervisor; dịch xy thêm 500 m để ánh xạ vào hệ tọa độ mô hình.
- Độ cao 30 m, vận tốc tối đa 5 m/s, gia tốc mỗi trục tối đa 2 m/s².
- Quan sát: vị trí ngang (2), vận tốc (2), pin (1), thời gian (1), vị trí tương đối
  của bốn nút (8), hiệu suất phổ mô hình (4), rồi xử lý bằng VecNormalize mới.
- Action là gia tốc chuẩn hóa. Bước PPO 1 s được nội suy qua 125 bước Webots 8 ms.
- Reward ưu tiên giữ đủ ba client và server trong bán kính 150 m; phạt vượt vùng,
  giảm khoảng cách tới nút xa nhất và hạn chế chuyển động dư thừa.
- Cất cánh được lập trình riêng, không tính vào pin/episode học. Episode dài 200 s;
  kết thúc thì giữ vị trí và tiếp tục xử lý relay, không tự đặt lại pin hoặc teleport.

Đây vẫn là điều khiển động học qua Supervisor, không phải điều khiển lực đẩy.
Chất lượng kênh đầu vào dùng công thức trong môi trường, chưa nhận từ ns-3.

## Gói UDP

Client gửi đến `127.0.0.1:8081`. Controller lấy vị trí nguồn theo DEF, kiểm tra khoảng cách 3D ≤150 m rồi gửi đến server `:9000` và bộ ghi log `:8080`. Hai trường `via_relay` và `relay_node` được gửi để tương thích cả Python và chương trình ghi log C++ hiện có. Gói JSON hỏng hoặc nguồn không xác định được bỏ qua. Mỗi vòng lặp xử lý tối đa 32 gói để tránh chiếm toàn bộ thời gian điều khiển.

## Kiểm tra ngoại tuyến

Từ thư mục dự án:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Các kiểm tra dùng checkpoint thật và Webots/socket giả lập cho vòng lặp: đủ 18 chiều quan sát, ý nghĩa gia tốc, giới hạn vận tốc/vùng, kết thúc episode, cất cánh, chu kỳ một giây, PPO tạo chuyển động và relay chịu được gói lỗi. Chúng không xác nhận hoạt động GUI hoặc mạng UDP thật.

## Chạy controller ngoài bằng terminal

Mở `VD2_vscode.wbt` bằng Webots và nhấn Run. Trong terminal tại thư mục dự án:

```bash
WEBOTS_HOME=/usr/local/webots \
PYTHONPATH="/usr/local/webots/lib/controller/python${PYTHONPATH:+:$PYTHONPATH}" \
LD_LIBRARY_PATH="/usr/local/webots/lib/controller${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
.venv/bin/python -u drone-rl/webots_world/controllers/dieukhien/dieukhien.py
```

Nếu chỉ có `PPO LOADED` mà không có `PPO READY`, kiểm tra đã mở đúng world `<extern>` và nhấn Run. Nếu thiếu DEF, dùng world VD2 đã cập nhật. Nếu bind cổng 8081 thất bại, dừng controller cũ trước khi khởi động lại. Lỗi display/Qt cần chạy Webots trong phiên desktop truy cập được.

## Chẩn đoán ba client mất kết nối

Client phát gói ngay lần đầu rồi mỗi 2 giây **mô phỏng** qua `robot.getTime()`;
không còn phụ thuộc `time.time()`. Controller UAV in `[LINK]` mỗi 5 giây mô phỏng:

- `IN_RANGE` / `OUT_OF_RANGE`: khoảng cách có đáp ứng ngưỡng 150 m hay không.
- `NO_PACKETS_YET`: relay chưa nhận gói từ client này.
- `RX_OK`: relay nhận gói trong 6 giây mô phỏng gần nhất.
- `RX_STALE`: đã từng nhận nhưng quá 6 giây chưa nhận gói mới.

Trạng thái nhận tại UAV không phải xác nhận server đã nhận; hiện không có ACK đầu cuối.
Nếu cả ba đều `NO_PACKETS_YET`, kiểm tra client controller có chạy và cổng 8081 có
đúng tiến trình nhận. Nếu `RX_OK OUT_OF_RANGE` (hoặc thứ tự log ngược lại), client
vẫn gửi nhưng relay bỏ gói do khoảng cách; kiểm tra quỹ đạo PPO.

Checkpoint relay mới đã được kiểm tra hình học 200 giây trên đúng tọa độ world,
với vị trí nội suy mỗi 8 ms: cả ba client trong bán kính 150 m suốt lượt chạy.
Trên 64 seed giữ riêng có nhiễu vị trí, trung bình giữ cả ba client đạt 99,15%;
64/64 lượt giữ cả ba trong vùng suốt 50 giây cuối. Một số trạng thái bắt đầu ngoài
vùng cần thời gian phục hồi. Đây chưa phải kết quả gói UDP/Webots GUI thật.

Mô hình cũ trước khi huấn luyện lại làm client 3 ra ngoài vùng sau khoảng 1,536 s;
kết luận đó không áp dụng cho checkpoint relay mới. Nếu có `NO_PACKETS_YET` dù
`IN_RANGE`, kiểm tra việc khởi chạy client/controller và socket, không quy ngay
cho PPO. Chỉ hỗ trợ độ cao đã huấn luyện 30 m; muốn đổi cần đánh giá/huấn luyện lại.

Tái lập chẩn đoán:

```bash
.venv/bin/python scripts/check-ppo-coverage.py
```
