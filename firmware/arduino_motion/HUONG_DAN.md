# Arduino UNO + 2 TB6600 + 2 stepper; Arduino Mega + 2 encoder REP

## Dùng trực tiếp trong ứng dụng đo IMU

Bản ứng dụng 2.1 có màn hình **Kiểm tra với encoder**, mở từ thanh trên cùng
hoặc chạy `IMUJointViewer.exe --bench`. Chọn riêng ba cổng: bộ thu IMU, Mega
đọc encoder, Uno điều khiển motor. Cổng Uno là tùy chọn; chỉ đo/ghi góc thì
không cần kết nối Uno. Tất cả dùng 115200 baud.

1. Nạp lại sketch Mega trong thư mục này. Bản mới xuất **100 Hz**, có số thứ tự
   mẫu và timestamp micro giây lấy cùng lúc chụp bộ đếm. Uno vẫn dùng các lệnh
   `M`, `C`, `S` hiện có; không cần đổi code motor.
2. Đóng Serial Monitor để ứng dụng mở được các cổng. Chọn hai IMU, kênh
   encoder 1 hoặc 2, số đếm/vòng 2500, trục quay X/Y/Z trên IMU đoạn trên
   và chiều góc IMU +1/−1 phù hợp với chiều đếm encoder.
3. Kết nối, giữ cơ cấu đứng yên 3 giây rồi **Lấy mốc 0°**. Mốc này chỉ lưu
   trong ứng dụng, không gửi lệnh `Z` làm thay đổi bộ đếm Mega.
4. **Bắt đầu ghi phép thử**, nhập góc hai motor rồi chọn **Quay một lần**
   hoặc **Quay lặp**. Kết nối thiết bị không tự làm motor quay.
5. Gửi **Dừng mềm / về gốc chu kỳ** và đợi Uno báo đã dừng. Sau đó dừng
   phép thử, xuất CSV kèm dữ liệu gốc, rồi ngắt kết nối.

Ứng dụng dùng số đếm có dấu làm góc chuẩn; không dùng góc lệnh motor.
Mỗi phép thử đối chiếu một cặp IMU với một kênh encoder; raw lưu cả hai kênh.
RMSE, độ lệch trung bình và lỗi lớn nhất không được làm giảm bằng cách sửa
đỉnh theo encoder hoặc tự fit độ trễ. Hướng trục và chiều quay cần kiểm tra
bằng một động tác nhỏ trước khi ghi phép thử chính.

**Giới hạn thời gian:** Mega và bộ thu IMU chưa có dây đồng bộ phần cứng.
Ứng dụng ghép đồng hồ theo thời điểm nhận USB và cố định mốc khi lấy zero.
Kết quả có thể chịu trễ USB, lệch đồng hồ và trễ fusion của BNO055; chưa đủ
bằng chứng để khẳng định sai số dưới 1°. Mất dữ liệu hợp lệ quá 500 ms sẽ
dừng ghi và yêu cầu motor dừng mềm. Lệnh này không thay thế nút dừng khẩn cấp
của cơ cấu. Khi đóng ứng dụng, lệnh S được gửi nếu Uno đang kết nối; cần
quan sát cơ cấu vì ứng dụng không thể xác nhận dừng sau khi đóng cổng.

## 1. Điều khiển hai stepper bằng Arduino UNO

Phải dùng **hai TB6600**: mỗi driver chỉ điều khiển một motor lưỡng cực. Không đấu hai motor song song vào một TB6600 nếu cần góc độc lập.

### Đấu phần tín hiệu mặc định (kiểu common-cathode)

| Arduino UNO | TB6600 số 1 | TB6600 số 2 |
|---|---|---|
| GND | PUL-, DIR- | PUL-, DIR- |
| D2 | PUL+ | - |
| D5 | DIR+ | - |
| D3 | - | PUL+ |
| D6 | - | DIR+ |

`ENA+` và `ENA-`: để hở; đa số module mặc định enable. Nếu module của bạn bắt buộc ENA, kiểm tra đúng manual in trên module vì clone TB6600 không đồng nhất.

Nếu đổi sang common-anode (`PUL+`, `DIR+` vào 5V; cực `-` vào chân Uno), đổi `STEP_PULSE_ACTIVE_LOW` thành `true`.

### Đấu nguồn và motor

- Mỗi TB6600: `VCC/DC+` và `GND/DC-` nối nguồn motor 12-24 V phù hợp. Không cấp motor từ chân 5V Arduino.
- Dùng đồng hồ thang ohm tìm hai cặp cuộn dây motor. Một cặp vào `A+/A-`, cặp kia vào `B+/B-`. Màu dây 17HS4401S thay đổi theo nhà bán; không đoán màu.
- Chỉnh dòng TB6600 theo nhãn motor và bảng DIP in trên đúng driver. Bắt đầu khoảng 1.5-1.7 A; tắt nguồn trước khi đổi DIP.
- Motor 1 quay 45° khi lệnh 90° cho thấy driver đang ở `1/16`. Code hiện đặt `MICROSTEPS = 16`; đặt cả hai TB6600 về `1/16` theo bảng DIP in trên module.

### Tỉ số truyền trong ảnh

Ảnh ghi `gearRatio = 1.0` nhưng chú thích "1/4": hai dữ kiện mâu thuẫn. Code dùng tên rõ nghĩa:

- Truyền trực tiếp: `MOTOR_TURNS_PER_OUTPUT_TURN = 1.0`.
- Giảm tốc 4:1, motor quay 4 vòng để trục ra quay 1 vòng: đặt `4.0`.
- Tăng tốc 1:4, motor quay 1 vòng làm trục ra quay 4 vòng: đặt `0.25`.

### Chạy

1. Arduino IDE > Library Manager: cài **AccelStepper** của Mike McCauley. `MultiStepper` nằm trong cùng thư viện.
2. Mở `uno_dual_tb6600.ino`, chọn Arduino Uno, upload.
3. Mở Serial Monitor: `115200 baud`. Code nhận cả `Newline` lẫn `No line ending`.
4. Gửi `M 90 -45`: motor 1 quay tương đối +90°, motor 2 quay tương đối -45°. Code chia tỷ lệ tốc độ và gia tốc để hai motor kết thúc gần như cùng lúc, không dừng gấp.
5. Gửi `C 90 90`: cả hai motor dao động từ vị trí hiện tại đến +90° rồi về gốc, lặp liên tục. Chế độ này dùng profile sin/cos: vận tốc bằng 0 ở biên, gia tốc liên tục khi đảo chiều, nên giảm lực khựng tại biên. Góc hai motor độc lập, ví dụ `C 90 -45`.
6. Gửi `S` hoặc `STOP`: hoàn thành nhịp đang chạy và quay mềm về gốc chu kỳ rồi dừng.
7. Nếu chiều sai, đổi `M1_REVERSE_DIRECTION` hoặc `M2_REVERSE_DIRECTION` thành `true`.
8. Nếu hụt bước/rung, giảm `MAX_SPEED_STEPS_PER_SECOND` từ `500` xuống `300` hoặc thấp hơn.

Lệnh `M` dùng acceleration/deceleration của `AccelStepper`; lệnh `C` dùng profile sin/cos riêng để không tạo điểm đổi gia tốc đột ngột tại biên. Hai thông số cần chỉnh theo cơ khí: `MAX_SPEED_STEPS_PER_SECOND` và `MAX_ACCELERATION_STEPS_PER_SECOND2`. Hạ acceleration trước nếu biên còn giật. Code điều khiển hở: góc là góc lệnh tương đối, không tự phát hiện mất bước.

## 2. Đọc hai encoder RA48HG-4WLN-G8JJ bằng Arduino Mega, không IC vi sai

Ảnh cung cấp chỉ xác nhận: lỗ trục âm phi 8 mm, nguồn DC 5 V, 2500 PPR, tốc độ danh định 6000 rpm và các cặp vi sai `A/A-`, `B/B-`, `Z/Z-`, `U/U-`, `V/V-`, `W/W-`. Ảnh không có pin connector hoặc bảng màu dây.

### Đấu tối thiểu, đọc single-ended

Dùng nguồn 5 V ổn áp riêng cho hai encoder; nối `0V` nguồn với `GND` Mega.

| Encoder | Tín hiệu | Arduino Mega |
|---|---|---|
| 1 | A+ qua điện trở nối tiếp 1 kOhm | D2 |
| 1 | B+ qua điện trở nối tiếp 1 kOhm | D4 |
| 2 | A+ qua điện trở nối tiếp 1 kOhm | D3 |
| 2 | B+ qua điện trở nối tiếp 1 kOhm | D5 |
| Cả hai | +5V | Nguồn 5 V ổn áp |
| Cả hai | 0V/GND | GND nguồn và GND Mega |

- **Không nối A-, B-, Z-, U-, V-, W- xuống GND.** Bọc cách điện từng dây không dùng; các dây này vẫn là ngõ ra chủ động.
- `Z`, `U`, `V`, `W` và các tín hiệu đảo không cần cho phép đo góc tương đối A/B.
- Shield nối chassis/PE ở một đầu; không dùng shield làm dây 0 V.
- Dùng cáp ngắn, xoắn A+ với 0 V và B+ với 0 V; tránh dây motor/TB6600. Không có receiver vi sai nên chống nhiễu kém.
- Trước khi nối Mega, đo A+ và B+ so với 0 V khi quay chậm: phải nằm trong khoảng 0-5 V. Nếu vượt 5 V, dừng; không nối vào Mega.

Catalog REP cùng họ thường dùng đỏ = +V, đen = 0V, xanh lá = A+, nâu = A-, trắng = B+, xám = B-, vàng = Z+, cam = Z-. Đây **không phải pinout đã xác minh riêng cho hậu tố G8JJ**. Chỉ dùng màu này khi tem dây/manual/người bán xác nhận.

### Chạy và độ phân giải

1. Mở `mega_dual_encoder.ino`, chọn Arduino Mega or Mega 2560, upload.
2. Serial Monitor: `115200 baud`.
3. Mặc định xuất: `ENC,seq,t_us,encoder1_count,encoder2_count`, 100 Hz. Dòng
   giới thiệu: `ENCODER,2,2500,100`. `t_us` tràn sau khoảng 71,6 phút;
   ứng dụng xử lý tràn bộ đếm và phân biệt với khởi động lại.
4. Gửi `Z` để đặt vị trí hiện tại của cả hai encoder là 0°; `Z1`/`Z2` đặt riêng; `P` in ngay.
5. Nếu chiều bị ngược, đổi `ENCODER1_SIGN` hoặc `ENCODER2_SIGN` thành `-1`.
6. Gửi `L` để dùng lại CSV năm cột cũ; `F` để về định dạng có timestamp.
   Không đổi định dạng hoặc zero trên Mega trong lúc đang ghi phép thử.
   Ứng dụng còn đọc được firmware Mega cũ 10 Hz; cần tăng khoảng nội suy
   tối đa, chẳng hạn 150 ms. Nên dùng bản 100 Hz cho thử nghiệm động.

Code đếm x1: `2500 count/vòng`, nên độ phân giải `360/2500 = 0.144°/count`. Encoder incremental không biết góc tuyệt đối sau khi mất nguồn; phải zero/homing lại.

### Giới hạn tốc độ quan trọng

Ở 6000 rpm, mỗi kênh A tạo `2500 x 6000 / 60 = 250 kHz`. Hai encoder tạo tổng 500.000 interrupt/s; Mega 16 MHz không thể đếm tin cậy bằng `attachInterrupt()` ở tốc độ này. Bản không-IC phù hợp quay chậm/thử bàn. Bắt đầu dưới 300 rpm và kiểm tra sai số sau nhiều vòng. Muốn đọc gần 6000 rpm có chiều quay: cần receiver RS-422/line-driver đúng chuẩn và bộ đếm quadrature phần cứng hoặc MCU nhanh hơn.

## 3. Thứ tự thử an toàn

1. Tháo tải cơ khí; tắt nguồn khi đấu dây hoặc đổi DIP.
2. Thử từng TB6600/motor ở lệnh `M 10 0`, rồi `M 0 10`.
3. Thử từng encoder quay tay, xác nhận một vòng gần đúng 2500 count.
4. Sau đó mới chạy hai motor và hai encoder cùng lúc.

## Cập nhật kiểm tra firmware ngày 19/10/2026

- Uno bỏ toàn bộ dòng lỗi khi quá dài hoặc có ký tự NUL/điều khiển; phần đuôi
  không còn có thể bị hiểu thành lệnh motor. Sau dòng lỗi, vẫn nhận lệnh mới.
- Mỗi vòng lặp chỉ đọc tối đa 32 byte lệnh trên cả Uno và Mega để dành lượt
  phục vụ motor/lấy mẫu. Đây không phải bảo đảm thời gian thực khi UART nghẽn.
- Uno giới hạn mỗi motor tối đa 500 xung/giây trong chế độ C, kể cả sau vòng
  lặp bị chậm. Không phát xung dồn để đuổi theo vị trí tính toán.
- Lệnh S lặp lại không kéo dài thời điểm dừng. Sau biên chu kỳ đã chọn, giữ
  đích tại gốc đến khi bộ đếm xung trở về gốc, kể cả khi bỏ lỡ đúng thời điểm biên.
- Mốc chu kỳ được cập nhật sau mỗi chu kỳ để vượt tràn micros() khi chạy lâu.
- Đã chạy test mã sketch bằng đồng hồ/GPIO mô phỏng và build bằng AVR GCC.
  Test này không kiểm chứng mất bước, nhiễu encoder hay sai số góc thực tế.

Góc motor vẫn là góc lệnh hở; dùng số đếm encoder làm tham chiếu đo. Profile
cosine giới hạn tốc độ/gia tốc danh định khi vòng lặp theo kịp; giới hạn xung
không bảo đảm gia tốc thực tế khi vòng lặp bị chậm hoặc đổi chiều để về gốc.
S là lệnh dừng theo nhịp, không phải dừng khẩn cấp. Firmware Uno hiện chưa có
watchdog liên lạc: mất USB hoặc ứng dụng tắt đột ngột không tự dừng chế độ C.

## Nguồn kỹ thuật

- Toshiba TB6600HG: một IC/driver cho một stepper hai pha; hỗ trợ 1/8 step; xung CLK tối thiểu 2.2 us: https://toshiba.semicon-storage.com/us/semiconductor/product/motor-driver-ics/stepping-motor-driver-ics/detail.TB6600HG.html
- Arduino Mega 2560: 16 MHz; các chân external interrupt gồm D2, D3, D18-D21: https://store.arduino.cc/products/arduino-mega-2560-rev3
- AccelStepper/MultiStepper: MultiStepper phối hợp nhiều motor đến vị trí đích cùng lúc bằng tốc độ hằng: https://github.com/jrv/AccelStepper
- Catalog REP/Rep-Avago 2012 dùng để đối chiếu ngõ ra line-driver và bảng màu dây của cùng họ sản phẩm: https://e-pwr.net/PDF/repavago/RepAvago-2012.pdf
