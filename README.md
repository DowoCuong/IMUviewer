# IMU Viewer — bản hợp nhất

Mở **`RUN_IMU.cmd`** trong thư mục này, hoặc chạy
**`dist/IMUJointViewer/IMUJointViewer.exe`**. Đây là portable hiện hành duy nhất;
giữ nguyên thư mục `_internal` cạnh EXE. Không cần cài lại ứng dụng.

`CURRENT.json` chỉ tới portable hiện hành. Phiên bản hiển thị trên thanh tiêu đề
cửa sổ; `release-manifest.json` cạnh EXE ghi phiên bản, hash nguồn và hash toàn bộ
file portable. Mã nguồn làm việc là `imu_desktop/` trong chính thư mục này.

## Ba màn hình trong cùng một ứng dụng

| Màn hình | Công việc |
|---|---|
| Gait / mô hình chân | Bốn IMU: chậu, đùi, cẳng chân, bàn chân; đủ ba thành phần góc mỗi khớp; mốc trung tính, trục chức năng; chu kỳ từ IC/TO xác nhận ngoài |
| Chi trên / phân tích nâng cao | Bốn IMU: thân, cánh tay, cẳng tay, bàn tay; chọn tay trái/phải; vai, khuỷu, cổ tay; hoặc mô hình chi dưới; hiệu chuẩn, kiểm tra độc lập, CSV/session/replay |
| Kiểm tra với encoder | Chọn cặp IMU, trục và chiều; encoder làm tham chiếu; RMSE trực tiếp và thống kê; có thể để trống encoder/motor để chỉ xem mạng bốn node |

Chuyển màn hình bằng liên kết phía trên. Dừng/ngắt kết nối chế độ hiện tại trước
khi chuyển để giữ quyền dùng COM rõ ràng. CLI cũ vẫn dùng được trên **cùng EXE**:

```powershell
& '.\dist\IMUJointViewer\IMUJointViewer.exe' --bench
& '.\dist\IMUJointViewer\IMUJointViewer.exe' --motion
```

## Chất lượng dữ liệu và giới hạn

Firmware gait hiện tại dùng IMU1 nhị phân, QUALITY_DATA type11 và HEALTH type12.
Chọn đúng COM hub; không tự chọn cổng khi đang cắm nhiều board. App kiểm tra
valid/state/calibration, tuổi chất lượng, session/epoch và giới hạn nội suy.
Các gói chưa FULL vẫn có thể xuất hiện trong dữ liệu gốc, nhưng không tự trở thành
mẫu đo định lượng hợp lệ. Tương thích firmware cũ phải bật rõ `--legacy-quality`.
Xem `PROTOCOL.txt` và `QUALITY.txt`; giao thức Sleepify có type11 khác, không dùng
lẫn như firmware gait.

Quaternion Hamilton `[w,x,y,z]`, sensor-to-segment và góc tương đối parent-child.
Chi dưới dùng Cardan Z-X-Y theo profile; quy ước chi trên và giới hạn kỳ dị ở
`UPPER_LIMB.md`. Mẫu đo không bị abs/clamp/dead-zone, không tự sửa theo encoder.
Mô hình 3D chỉ minh họa. FULL và test phần mềm không chứng minh RMSE <1° trên
mọi góc hoặc khi đeo trên người. Cần kiểm chứng cách gắn, trục, thời gian, tốc độ,
nhiệt độ và tham chiếu độc lập. Timestamp là hoàn tất đọc I2C, không phải timestamp
fusion nội bộ của BNO055.

Không có encoder thì không có RMSE. Ghép encoder–hub qua PC vẫn có bất định USB,
độ trễ BNO và drift; không fit lag/offset ngay trên phép thử rồi thay số RMSE trực
tiếp. Node tạo tải không có quaternion, không tham gia góc hoặc RMSE. Thống kê
gói/gap từ luồng nhận không được gọi là RF loss thuần.

Motor là tùy chọn; kết nối không tự làm motor quay. Lệnh S của Uno là dừng mềm,
không phải dừng khẩn cấp. Nâng cấp app không tự flash firmware hoặc chạy motor.
Firmware/Arduino có README và manifest riêng trong `firmware/`; bộ nRF mới hơn
được quản lý tại nguồn `D:/IMU ESB/NRF52840/Firmware/supermini_esb`.

## Chạy từ nguồn và kiểm tra

Môi trường có sẵn: `.venv310/Scripts/python.exe`, PySide6 và pyserial.

```powershell
.\.venv310\Scripts\python.exe run_desktop.py
.\.venv310\Scripts\python.exe run_desktop.py --bench
.\.venv310\Scripts\python.exe run_desktop.py --motion
$imuTestTemp = Join-Path $PWD ('build/pytest-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
.\.venv310\Scripts\python.exe -m pytest -p no:cacheprovider --basetemp $imuTestTemp -q
```

Các tools smoke/benchmark dùng dữ liệu mô phỏng, không chứng nhận độ chính xác
phần cứng. Không chạy thử trên COM/motor thật để kiểm tra đóng gói.

## Build và quay lui

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\packaging\build_windows.ps1
```

Mặc định chỉ build portable, không cần Inno Setup. Build trong thư mục staging,
kiểm tra đủ ba màn hình và ghi manifest trước khi đổi bản hiện hành. `dist` cũ
được chuyển vào `backups/portable-<thời gian>`; build thất bại trước khi xuất bản
không thay bản đang dùng. Đóng EXE hiện hành trước khi xuất bản build mới.

Chỉ khi cần bộ cài mới dùng `-Installer`; phiên bản bộ cài lấy từ cùng manifest,
không khai báo một số phiên bản riêng. Phiên bản nguồn duy nhất nằm trong
`imu_desktop/__init__.py`; pyproject đọc động giá trị này.

Các bản portable lịch sử được giữ tại
`backups/viewer-consolidation-20261116/previous-builds/`.
Nguồn và README trước hợp nhất ở `backups/viewer-consolidation-20261116/source/`.
Không dùng các bản quay lui như một bản hiện hành thứ hai.
Hồ sơ đối chiếu/hash/test hợp nhất ở `exports/verification/viewer-consolidation-20261116/`.

Dữ liệu xuất và profile hiệu chuẩn đã có được giữ nguyên. CSV/session/replay
giữ định dạng hiện tại; thay đổi hợp nhất không sửa engine góc hoặc firmware.
Thông tin thư viện bên thứ ba xem `THIRD_PARTY_NOTICES.md`.
