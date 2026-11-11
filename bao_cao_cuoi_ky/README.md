# Báo cáo cuối kỳ — cập nhật PPO relay ba e-puck

Đã cập nhật trực tiếp các tệp Word, PDF và Markdown cùng tên. PDF hiện có 28 trang, 7 hình và 11 bảng; giữ cấu trúc báo cáo học phần gồm bìa, mục lục, 7 chương, tài liệu tham khảo và phụ lục.

Nội dung mới dùng checkpoint relay: độ cao 30 m, vận tốc 5 m/s, gia tốc 2 m/s² mỗi trục, 327.680 bước học, checkpoint chọn tại 262.144. Kết quả dựa trên 64 seed giữ riêng; chỉ số chính là tỷ lệ thời gian đủ ba client trong vùng 150 m. Chưa có số đo UDP đầu cuối hoặc Webots GUI xác nhận.

Hình được dựng từ dữ liệu trong `minh_chung/relay_3epuck`. Tái tạo:

```bash
python3 nguon/ve_hinh.py
python3 nguon/xuat_bao_cao.py
```

Nguồn nội dung: `nguon/noi_dung.py`. Các script và dữ liệu thực nghiệm cũ còn lưu để đối chiếu, không dùng làm kết quả checkpoint mới. Bản báo cáo trước cập nhật nằm trong `luu_tru_truoc_cap_nhat_PPO`.

Đã xem bố cục PDF qua ảnh render và kiểm tra XML của DOCX. Chưa render DOCX trực tiếp trong Word; cập nhật mục lục Word bằng F9 nếu cần. Thông tin trường, giảng viên, thành viên và đóng góp chưa được xác nhận vẫn để trống.
