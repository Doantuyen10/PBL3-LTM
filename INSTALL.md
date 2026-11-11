# Cài đặt và chạy PBL3-LTM trên máy khác

Hướng dẫn cho Ubuntu/Linux hoặc WSL có phiên desktop chạy được Webots. Python 3.10 được dùng trong môi trường dự án. Cài Webots R2025a từ bản phân phối chính thức của Cyberbotics. Nếu dùng WSL, cần WSLg hoặc X server hoạt động; đặt DISPLAY đơn thuần chưa bảo đảm GUI chạy được.

## 1. Lấy mã và tạo môi trường Python

Sau khi clone repository, mở terminal tại thư mục gốc `PBL3-LTM`:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements.txt
cp .env.example .env
```

Nếu thiếu module venv trên Ubuntu, cài gói `python3-venv` bằng trình quản lý gói của máy. Không cài gói PyPI tên `controller`; API này đi kèm Webots. Runtime không cần GPU hoặc ns-3 để thử relay UDP.

`requirements.txt` khai báo khoảng phiên bản để pip giải quyết phụ thuộc trên máy mới. `requirements-observed.txt` ghi phiên bản tại lúc huấn luyện để tra cứu, không phải lockfile bảo đảm cài được trên mọi máy. Nếu checkpoint không nạp được do khác phiên bản, đối chiếu traceback và các phiên bản đã ghi.

## 2. Sửa .env

```dotenv
WEBOTS_HOME=/usr/local/webots
PYTHON_EXECUTABLE=.venv/bin/python
WEBOTS_WORLD=drone-rl/webots_world/worlds/VD2_vscode.wbt
```

`WEBOTS_HOME` là thư mục cài Webots, chứa executable `webots` và thư mục `lib/controller`. Đường dẫn tương đối được tính từ gốc repository, không phải thư mục terminal hiện tại. Đặt đường dẫn có dấu cách trong dấu nháy. Không dùng `$PWD`, `${...}` hoặc lệnh shell trong file này. Biến đã export trong terminal được ưu tiên hơn .env.

Launcher `scripts/run_project.py` đọc .env và thiết lập đường dẫn API/thư viện cho controller. Chạy trực tiếp dieukhien.py hoặc cấu hình F5 cũ không tự đọc .env; dùng các lệnh dưới đây khi chuyển máy.

Độ cao 30 m và bán kính 150 m là cấu hình gắn với checkpoint đã huấn luyện, không đưa thành biến tùy ý để tránh thay đổi bài toán mà không đánh giá lại.

## 3. Kiểm tra checkpoint và chạy

Các tệp sau phải có trong `drone-rl/webots_world/controllers/dieukhien/`:

- ppo_relay_3epuck.zip
- vecnorm_relay_3epuck.pkl
- relay_model.json
- relay_env.py và uav_bs_env.py
- ppo_bridge.py và dieukhien.py

Kiểm tra lệnh trước khi chạy:

```bash
python3 scripts/run_project.py webots --dry-run
python3 scripts/run_project.py controller --dry-run
```

Terminal thứ nhất:

```bash
python3 scripts/run_project.py webots
```

Trong Webots nhấn **Run**. World này dùng controller `<extern>`. Terminal thứ hai, tại gốc dự án:

```bash
python3 scripts/run_project.py controller
```

Chờ khoảng 6 giây mô phỏng để UAV lên 30 m. Log mong đợi: `PPO LOADED`, `PPO READY`, `PPO ACTIVE`, `PPO STEP`, `UDP RELAY`. Các client phát mỗi 2 giây mô phỏng. Sau 200 bước PPO, UAV giữ vị trí cuối và tiếp tục relay. Để chạy episode mới, dừng controller, reset world rồi khởi động lại controller. Chỉ mở một world controller ngoài tại một thời điểm.

Nếu cần chạy bộ kiểm tra ngoại tuyến:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Các phép kiểm tra này không thay thế việc chạy Webots GUI và kiểm tra gói tại server.

## 4. Những tệp đưa lên Git

Commit `.env.example`, `requirements.txt`, `requirements-observed.txt`, `INSTALL.md`, `scripts`, các world/controller, `.vscode` nếu cần và đầy đủ bộ checkpoint relay ở trên. Các tệp mô hình hiện đủ nhỏ để dùng Git thông thường.

Không commit `.env`, `.venv/`, `__pycache__/`, `logs/` và cây nguồn/build `tools/`; .gitignore đã khai báo. Metadata model có các đường dẫn nguồn/đánh giá trên máy tạo mô hình để truy vết; controller không dùng các đường dẫn đó để nạp model. Model được nạp từ thư mục chứa controller.

Nếu repository đang dùng cơ chế chuyển đổi line ending, giữ LF cho `relay_env.py` và `uav_bs_env.py`: controller kiểm tra SHA-256 của mã môi trường. File .gitattributes đã cố định LF cho các tệp này.

Mã huấn luyện hiện nằm ngoài repository ở `../PPO/relay_3epuck`. Clone riêng PBL3-LTM đủ để chạy checkpoint, chưa chứa bộ huấn luyện và lịch sử học. Nếu cần chia sẻ cả phần huấn luyện, đưa thư mục đó vào repository riêng hoặc vị trí đã thống nhất và cập nhật các đường dẫn dự án.

## 5. Xử lý lỗi

- Không tìm được Webots/API: sửa WEBOTS_HOME trong .env.
- Lỗi Qt/display: chạy trong phiên desktop có màn hình; kiểm tra phụ thuộc Qt báo thiếu.
- Model/hash không khớp: lấy lại trọn bộ checkpoint và hai tệp môi trường tương ứng; không ghép model mới với normalizer cũ.
- Cổng 8081/9000 đang dùng: dừng phiên controller/server cũ trước khi chạy lại.
- IN_RANGE nhưng NO_PACKETS_YET: kiểm tra client controller có chạy và lỗi console Webots.
- Không tải được PROTO: world có EXTERNPROTO qua Internet; máy cần mạng hoặc cache tài nguyên Webots.
