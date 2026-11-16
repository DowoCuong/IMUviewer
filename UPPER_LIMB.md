# Đo chi trên và nâng cấp phần xử lý chung — 2.3.1

Đây là bản phần mềm để kiểm chứng. Chưa đo trên board/người trong phiên phát
triển này; chưa chứng minh RMSE <1° cho mọi góc. Bản 2.3.1 dùng firmware QUALITY
với health/epoch mới; xem QUALITY.txt và PROTOCOL.txt. Các quy ước góc bên dưới
giữ nguyên từ 2.2.0; mặc định strict chặn node thiếu metadata chất lượng mới.

## Chạy portable

Giữ toàn bộ thư mục IMUJointViewer, gồm `_internal` cạnh EXE.
Chạy `Mo-chi-tren.cmd`, hoặc `IMUJointViewer.exe --motion`.
Màn hình cũ: EXE không tham số. Bench: `--bench`.

Màn hình mới chọn được chi trên hoặc chi dưới. Màn hình gait cũ giữ quy ước,
profile, cột CSV và phân tích IC–TO–IC đã bàn giao. Màn hình chi dưới nâng cao
có kết quả từng cặp; không tự biến dữ liệu từng cặp thành một chu kỳ gait đủ.

## Mapping và nghĩa phép đo

Chi trên: 1 thân ngực, 2 cánh tay trên, 3 cẳng tay gần cổ tay, 4 mu bàn tay.
Đo một bên tay; gắn chắc, tránh gá bắc qua khớp. Ghi vị trí/chiều gá và gắn lại
phải hiệu chuẩn lại. Thân–cánh tay là humerothoracic. Không có cảm biến bả vai
thì không đo riêng glenohumeral. Forearm IMU không tách radius/ulna.

Chi dưới nâng cao: 1 chậu, 2 đùi, 3 cẳng chân, 4 bàn chân; hiện chỉ khai báo
quy ước bên phải đã bàn giao. Không tự suy ra dấu cho chân trái.

## Quy ước đã implement

Quaternion Hamilton scalar-first `[w,x,y,z]`; q_AB ánh xạ vector frame B sang A.
`q_WB=q_WS*q_SB`, `q_PC=conj(q_WP)*q_WC`.
Frame khớp riêng: `q_J=conj(q_PJ)*q_PC*q_CJ`. Hai frame được lưu riêng.

Mọi frame là thuận tay phải. Tư thế vận hành: thân thẳng, tay buông, khuỷu duỗi
tự nhiên, lòng bàn tay hướng trước. Các frame đoạn được lấy mốc với x trước,
y trên/proximal, z phải. Đây là giả định tư thế vận hành, chưa xác định mốc
xương riêng của người đo. Không lấy mốc để xóa góc bất thường đã biết.

| Khớp | Phân rã intrinsic | Thành phần xuất | Chiều |
|---|---|---|---|
| Vai | Ry(a) Rx(b) Ry(c) | plane, elevation, axial | a,c tọa độ YXY kỹ thuật; b≥0 |
| Khuỷu/cẳng tay | Rz(f) Rx(d) Ry(r) | flexion, carrying, supination | f; s*d; -s*r |
| Cổ tay | Rz(f) Rx(d) Ry(r) | flexion, radial, axial_residual | f; -s*d; -s*r |

s=+1 tay phải, -1 tay trái. Z flexion dương ở cả hai bên. Sấp/ngửa và lệch
quay có dấu riêng theo bên. Không dùng phép phản chiếu làm quaternion.

Vai: plane/axial là chính các tọa độ a/c của YXY trong frame đã lưu. Chúng
không được tự đổi thành nhãn gấp/dạng hoặc thành góc vai lâm sàng độc lập.
Ở tay phải, nâng sang bên có thể cho plane gần ±180°, nâng ra trước gần -90°
với frame tư thế tham chiếu này. Cần dùng đúng quy ước khi đối chiếu tham chiếu.
Không tự cộng offset để khiến plane trông thuận mắt.

YXY: b=acos(Ryy), a=atan2(Rxy,Rzy), c=atan2(Ryx,-Ryz).
ZXY: d=asin(Rzy), f=atan2(-Rxy,Ryy), r=atan2(-Rzx,Rzz).
Clamp chỉ dùng chặn sai số làm tròn miền asin/acos.

Miền YXY: a,c trong [-180,180], b trong [0,180].
Miền ZXY: f,r trong [-180,180], d trong [-90,90]. Nhảy nhánh ±180° có thể xảy
ra; không unwrap tự động mọi tọa độ 3D hoặc đổi sequence giữa phiên.

Conditioning YXY dùng |sin(b)|; ZXY dùng |cos(d)|. Chi trên mặc định dưới 0.02
thì tọa độ đầu/cuối trả null, tọa độ giữa và quaternion vẫn giữ. Ngưỡng này là
tiêu chí kỹ thuật có trong profile, không là độ bất định hay chứng nhận <1°.
Chi dưới nâng cao giữ ngưỡng 1e-6 như engine cũ; không đổi ZXY hoặc dấu cũ.

Đây là mô hình frame vận hành tham khảo khuyến nghị ISB; KHÔNG tuyên bố
ISB-compliant khi chưa xác định đầy đủ mốc giải phẫu và kiểm chứng quy trình.
Nguồn: https://media.isbweb.org/images/documents/standards/isb_jcs_part_ii.pdf
Nghiên cứu hiệu chuẩn: https://pubmed.ncbi.nlm.nih.gov/26263993/

## Hiệu chuẩn và kiểm tra

1. Chọn đúng mô hình/bên, chọn hub và kết nối. Cần bốn IMU thật, đủ calibration.
2. Giữ tư thế tham chiếu ≥3 giây rồi lấy mốc. Cửa sổ cần ≥2 giây liên tục;
   độ lệch tối đa quanh quaternion trung bình không quá 1°.
3. Với mỗi chuyển động, bấm Ghi trục, bắt đầu chiều dương, lặp ~5 giây và
   Hoàn tất. Thực hiện sáu chuyển động: nâng trước, nâng bên, gấp khuỷu,
   ngửa cẳng tay, gấp cổ tay, lệch về ngón cái.
4. Khi ghi ngửa cẳng tay, giữ khuỷu gần duỗi như tư thế chuẩn; sau lấy mốc có
   thể sấp nhẹ để có khoảng ngửa theo chiều dương. Hạn chế xoay cánh tay.
5. Ghi lại từng chuyển động bằng nút Kiểm tra độc lập, dùng cửa sổ mới.
6. Chỉ sau khi đủ trục và đủ kiểm tra được chấp nhận, hàng góc mới được ghi.
   Xem trước vẫn hiện, nhưng được đánh dấu chưa đủ hiệu chuẩn.

Trục lấy từ hiệu vận tốc góc hai đoạn ở world frame rồi đưa về từng đoạn;
vận tốc suy từ quaternion fusion, KHÔNG phải gyro thô. Giữ mẫu chuyển động
≥10°/s, tối thiểu 30 mẫu chuyển động, biên độ ≥15°, thời lượng ≥2 giây.
PCA concentration ≥0.95. Chiều đầu giải dấu PCA.

Mỗi khớp cần hai hướng độc lập. Dùng z gấp và y ngửa cho khuỷu; z gấp và x
nâng bên/lệch quay cho vai/cổ tay. |cross| cần ≥0.7. Giữ nguyên trục z đo được,
trực chuẩn hướng thứ hai, lưu mức độc lập và góc thay đổi do trực chuẩn hóa.
Frame trên parent/child được tính riêng. Lưu trục đo ban đầu và concentration.

Kiểm tra dùng trục mới, khác cửa sổ khớp, sai khác tối đa hai frame ≤5°.
5° là ngưỡng lặp trục kỹ thuật ban đầu, KHÔNG là RMSE góc đo. Cần hiệu chỉnh
quy trình/ngưỡng bằng thí nghiệm. Khớp lại một hướng hủy kiểm tra của khớp đó.
Màn hình chi dưới nâng cao dùng phép khớp cũ và kiểm tra lặp frame ≤5°;
màn hình gait cũ bảo toàn chính sách chấp nhận đã bàn giao.

Một tư thế chuẩn không tự xác định đúng xương. Không chứng minh độ chính xác
giải phẫu từ việc quaternion tái tạo tư thế trung tính bằng 0.

## Thời gian, mất mẫu và chất lượng

Timeline lưới 10 ms trên clock hub, nội suy SLERP tối đa 30 ms, không ngoại suy.
Ba pair synchronizer dùng cùng lưới clock; thêm synchronizer đủ bốn node chỉ
cho cửa sổ hiệu chuẩn. Đây là lựa chọn tái sử dụng nhỏ, có chi phí lặp được
benchmark; chỉ gom buffer nếu profiler chỉ ra cần thiết.

Mỗi kết quả chỉ là một khớp tại time_us; không ghép hàng các thời điểm khác
nhau thành một frame đầy đủ. Mất node4: wrist hết hiệu lực, cặp khác còn được
tính khi đủ mẫu. BNO calibration giảm xóa buffer liên quan. Reset một node/hub
dừng cả phiên, hủy calibration; xuất rồi bắt đầu phiên mới.

CSV long format: t_ms, joint, các cột góc, model/side/calibration_id, quaternion
khớp, quality_json, clock uncertainty, interpolation gap và time basis.
Cột trống là null hoặc góc của khớp khác. Quaternion lưu cả gần kỳ dị.
Các lý do: calibration_required, near_singularity, missing_pair; mất/calibration
drop còn có số đếm và raw. Không gán một điểm confidence chưa kiểm chứng.

Coverage lấy số tick hợp lệ mỗi góc / số tick trong khoảng hub đã quan sát,
gồm cả khoảng chuẩn bị hiệu chuẩn. Tick bị mất không được nội suy quá giới hạn
và vẫn giảm coverage. Không suy coverage này thành RF loss thuần. Nếu tất cả
node tắt cùng lúc, không thể suy ra phần thời gian không còn quan sát được.

Không tự điều chỉnh heading theo khớp. Không có raw accel/gyro/mag để chạy
lại fusion đầy đủ. NDOF calibration 3/3/3/3 không chứng minh sai số <1°.
Nguồn BNO: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

## Replay, dữ liệu và hiệu năng

Dừng rồi xuất CSV và .session.zip. Bundle gồm raw.jsonl, results.jsonl,
angles.csv và metadata.json. History profile gắn vị trí packet; cửa sổ thu
hiệu chuẩn được lưu để replay không biến preview thành measured.

Replay trong UI chỉ nhận bundle màn hình mới (model_id và history đầy đủ).
Không tự đoán các điều kiện hiệu chuẩn còn thiếu của archive cũ.
CLI: python -m imu_desktop.core.replay_motion input.session.zip output.csv
Raw đi qua StreamDecoder, MotionSession và cùng engine như live; không mở COM.

Raw/góc ghi buffered vào tệp tạm; UI giữ ≤12000 hàng. Không có writer queue,
không fsync mỗi mẫu, không nén trong callback. I/O buffered vẫn có thể block
khi đĩa chậm: benchmark đường ghi, lỗi phải dừng/báo rõ. Chỉ thêm worker writer
có hàng đợi giới hạn nếu đo thực tế cho thấy I/O cản thu nhận. Không tuyên bố
mọi thiết bị lưu trữ đều đáp ứng 100 Hz từ benchmark trên một máy.

Gait cũ chuyển danh sách góc sang RowLog trên đĩa; export/summary dùng luồng.
Phân tích gait offline vẫn cần tải dữ liệu vào RAM theo thuật toán cũ; đây là
giới hạn riêng khi phân tích, không phải bộ nhớ đường thu nhận/export.
Bench cũ giữ cách lưu paired rows và các giới hạn vốn có; chưa viết lại bench.

Thống kê sai số trực tiếp:
python -m tools.summarize_motion_errors paired.csv --error-column error_deg
Tính RMSE/MAE/bias/p95 tuyệt đối bằng SQLite tạm để không giữ mọi sai số trong
RAM. Không fit lag/offset. Độ phủ dòng CSV khác độ phủ toàn thời gian đo.
Nếu có cột vận tốc tham chiếu đã kiểm chứng theo deg/s, thêm
`--speed-column ten_cot` để chia nhóm 0–30, 30–90, 90–180, >180°/s.
Không tự coi vận tốc lệnh motor là vận tốc tham chiếu.
Không có cột tham chiếu/sai số thì không có RMSE; không sinh số từ quaternion.

## Kiểm chứng phần cứng cần thực hiện tiếp

Không tự nạp/chạy motor. Sau khi người dùng chuẩn bị thiết bị:
- Gá cứng: từng trục, nhiều góc, nhiều vận tốc, nhiều hướng toàn cụm.
- Đo cross-talk trên trục không quay bằng tham chiếu độc lập.
- Đo đồng thời encoder/hub; hiệu chỉnh độ trễ chỉ từ trial độc lập.
- Tháo/gắn, nhiều người thao tác, nhiệt độ và drift theo thời gian.
- Trên người: đánh giá mô mềm và tham chiếu 3D phù hợp, cùng quy ước góc.
- Báo RMSE trực tiếp, MAE, bias, p95, coverage và bất định tham chiếu.
- Không kết luận đủ mọi góc từ một encoder một trục hoặc test toán học.
