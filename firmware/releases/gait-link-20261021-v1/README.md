# Firmware IMU gait — ổn định đường truyền, 21/10/2026

Đã đối chiếu firmware IMU hiện có với mã nguồn ESB8 ngày 12/10/2026.
Gói này dùng cho app IMU Joint Viewer 2.0/2.1 và bốn BNO055 trên một chân.

## Tệp cho từng board

| Board | UF2 |
| --- | --- |
| Hub USB | `imu-hub-accuracy-gait100.uf2` |
| Node 1 — chậu | `imu-tx-accuracy-gait100-n1.uf2` |
| Node 2 — đùi | `imu-tx-accuracy-gait100-n2.uf2` |
| Node 3 — cẳng chân | `imu-tx-accuracy-gait100-n3.uf2` |
| Node 4 — bàn chân | `imu-tx-accuracy-gait100-n4.uf2` |

Nạp hub và cả bốn node cùng bộ này: kênh và địa chỉ đã thay đổi. Mỗi board nhận
đúng một UF2 theo vai trò; không trộn với UF2 cũ hoặc firmware Sleepify.
Chưa có thao tác nạp hoặc mở cổng COM trong quá trình xây gói này.

## Cấu hình đã chọn

| Thành phần | Lựa chọn và lý do |
| --- | --- |
| Đường truyền | 2 Mbps, +8 dBm; giữ bitrate và tăng công suất theo ESB8 |
| Mạng gait | Kênh 80, địa chỉ riêng; tách khỏi mạng ESB8 kênh 40 |
| Tần số | 100 Hz mỗi IMU, 400 gói DATA/s tổng; frame 40 ms, slot 1 ms |
| Đồng hồ | HFXO luôn được giữ riêng; TIMER3 không đổi nguồn khi USB/ESB đổi trạng thái |
| Lịch radio | Hai lần khởi tạo vai trò mỗi frame bình thường; trả về PRX trước beacon |
| Lỗi truyền | Thử lại cấu hình trong khoảng nghỉ, không xóa clock fit chỉ vì mất ACK |
| Lỗi cấu hình | Chờ 100 ms rồi thử lại; giữ boot/session và chuỗi beacon |
| Đồng bộ nhiễu | Giữ fit cũ chỉ khi bất định đến cuối frame ≤50 µs; không gia hạn tuổi cặp tốt |
| USB | FIFO có interrupt; thoát khi không tiến triển 100 ms, đếm dữ liệu bị bỏ |
| Lệnh USB | Bỏ cả dòng lỗi; mất byte thì bắt lại tại dấu xuống dòng |
| Kiểm tra thời gian | Giữ deadline 700 µs, cổng slot 100 µs, hết hạn sync 120 ms, zero retries |

Giữ công thức quaternion, TIMED_DATA 64 byte, hiệu chuẩn/NVS và các ngưỡng
loại mẫu lỗi của engine hiện tại. Giữ cách gửi mẫu mới nhất, tránh kéo dài
độ trễ bằng hàng chờ mẫu cũ. Các phần PPG/áp suất của Sleepify không thuộc
giao thức đo khớp nên không được đưa vào gói gait.

Kênh 80 là lựa chọn để tách hai hệ thống đang có; chất lượng kênh thực tế vẫn
cần đo tại nơi dùng. Có thể đổi `CONFIG_IMU_RF_CHANNEL` đồng bộ trên mọi board.
+8 dBm và HFXO luôn bật làm tăng điện năng tiêu thụ so với cấu hình trước.

## Kiểm chứng và giới hạn

Xem `manifest.json` và `SHA256SUMS.txt`; mã nguồn tương ứng trong `source.zip`.
`rollback/` giữ năm UF2 trước thay đổi. Khi quay lại cũng dùng đủ một bộ.

Các test thực thi mã C với clock/USB/ESB mô phỏng: lỗi ACK, khởi tạo lỗi,
khởi tạo vai trò kéo dài 1 ms, lịch 100/400 Hz, USB đầy/đóng/mở lại, CRC,
HFXO không phụ thuộc USB, bước nhảy clock, tràn bộ đếm và chuỗi beacon lỗi.
Build dùng nRF Connect SDK 3.4.0; UF2 được kiểm tra vùng ứng dụng
`[0x26000, 0xEC000)`, không ghi vùng NVS/bootloader.

Chưa có phép đo RF/nguồn/COM trên board thật. Timestamp vẫn là thời điểm đọc
I²C hoàn thành, chưa xác định thời điểm fusion bên trong BNO055. Chưa đồng bộ
phần cứng Mega–hub. Những thay đổi này chưa chứng minh RMSE dưới 1°.
