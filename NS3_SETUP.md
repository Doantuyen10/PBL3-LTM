# Cài và chạy ns-3

Script cài ns-3.47 vào `tools/ns-3.47` trong dự án, bật ví dụ và kiểm thử.
Không cần sudo. Phiên bản được cố định để dễ tái lập môi trường.
Nguồn: https://www.nsnam.org/releases/ns-3-47/

Từ thư mục dự án, chạy trong terminal có mạng:

```bash
bash scripts/setup-ns3.sh
```

Script tải mã nguồn chính thức, giải nén, cấu hình CMake, biên dịch với 2 luồng,
chạy `hello-simulator`, `first` và bộ kiểm thử `core`. Có thể chỉnh số luồng:

```bash
NS3_JOBS=4 bash scripts/setup-ns3.sh
```

Nếu máy không tải trực tiếp được, tải tệp sau bằng trình duyệt và lưu thành
`tools/ns-3.47.tar.bz2`, rồi chạy lại script:
https://www.nsnam.org/releases/ns-3.47.tar.bz2

Sau khi cài thành công:

```bash
cd tools/ns-3.47
./ns3 run first
./ns3 run 'second --nWifi=3'
```

GCC 11.4, Python 3.10, CMake 4.4 và Ninja đã có trên máy tại thời điểm chuẩn bị.
Trong terminal của agent, truy cập nsnam.org và gitlab.com thất bại do không phân giải
được tên miền; chưa tải hoặc biên dịch thành công ns-3. Chạy script bên ngoài môi trường
bị giới hạn mạng để hoàn tất cài đặt.

Python bindings chưa bật; môi trường này chạy mô phỏng C++.
Kết nối Webots, PPO và MUNILS cần cấu hình riêng sau khi ns-3 chạy được.
