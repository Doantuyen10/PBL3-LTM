# PBL3-LTM
 Điều hướng và Lập quỹ đạo nhận thức truyền thông (Communication-Aware Trajectory Planning)
- Mô tả chủ đề: Sử dụng thuật toán DRL (ví dụ: DDPG, PPO) để điều khiển quỹ đạo bay của đàn UAV sao cho vừa hoàn thành nhiệm vụ vật lý (như bao phủ khu vực, tuần tra), vừa duy trì kết nối mạng ổn định với trạm mặt đất hoặc giữa các UAV.
- Điểm mạnh khi dùng MUNILS: MUNILS cho phép đánh giá mối quan hệ tương hỗ hai chiều: Khi UAV bay ra xa, ns-3 sẽ mô phỏng sự suy hao tín hiệu, trễ (delay) hoặc rớt gói tin (packet loss); từ đó tác động trực tiếp lên vòng lặp điều khiển chuyến bay thực tế trong PX4/Gazebo.   
- Bài toán cụ thể: Tối ưu hóa quỹ đạo đàn UAV để tối đa hóa dung lượng đường truyền (Throughput) hoặc giảm thiểu tối đa tỷ lệ rớt gói tin trong các vùng bị cản trở tín hiệu.
