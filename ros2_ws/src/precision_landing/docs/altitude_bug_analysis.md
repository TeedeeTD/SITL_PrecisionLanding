# Tóm Tắt Phân Tích Lỗi "Delay 5s" & Vấn Đề Quản Lý Độ Cao

Tài liệu này tổng hợp lại quá trình debug, phân tích nguyên nhân cốt lõi và tư duy thiết kế để giải quyết triệt để vấn đề quản lý độ cao (Altitude Management) đối với hệ thống Precision Landing, đặc biệt khi phải hạ cánh trên các bục đáp (pad) có chiều cao biến động.

## 1. Hiện Tượng (Symptom)
Hệ thống đã được trang bị logic phát hiện chạm đất tức thời (khi mức tụt độ cao thực tế chậm hơn 15cm so với vận tốc rơi mục tiêu). Tuy nhiên, thực tế drone vẫn **mất tới 5 giây để ngắt động cơ** khi chạm bục đáp.

## 2. Truy Tìm Nguyên Nhân Gốc Rễ (Root Cause)
Qua quá trình phân tích logic State Machine (FSM) và thông số cấu hình, nguyên nhân được làm rõ như sau:

> [!WARNING]
> **Xung đột giữa Cấu hình và Môi trường vật lý:**
> Tham số `final_alt` (ngưỡng độ cao để chuyển sang trạng thái nhắm mắt hạ mù `FINAL_APPROACH`) được đặt là `0.1m` (tức là ngưỡng kích hoạt là 0.15m).
> Tuy nhiên, **bục đáp thực tế lại cao 0.18m**.

1. **FSM bị kẹt cứng (Deadlock):** Code đang sử dụng độ cao tuyệt đối của drone (`pos_enu_.z`) làm điều kiện chuyển trạng thái. Khi drone đáp xuống bục, nó bị chặn vật lý ở độ cao `0.18m`.
2. **Logic chạm đất không được gọi:** Vì `0.18m` lớn hơn ngưỡng `0.15m`, FSM không bao giờ thỏa mãn điều kiện để đi vào `FINAL_APPROACH`. Nó bị kẹt mãi mãi ở trạng thái `DESCENT`.
3. **Phụ thuộc vào Fallback chậm chạp:** Do kẹt ở `DESCENT`, đoạn code phát hiện chạm đất 15cm ưu việt của chúng ta không được thực thi. Hệ thống phải chờ bộ Land Detector nội bộ của PX4 phát hiện chạm đất (mất 5s). Lúc đó PX4 gán cờ `LANDED_STATE_ON_GROUND`, FSM mới chịu nhảy sang `FINAL_APPROACH` và gọi disarm.

## 3. Bản Chất Cảm Biến PX4 EKF2 (Sự kiện Bục Cao 1 mét)
Một câu hỏi đặt ra: *"Nếu dùng Range Finder, khi lướt qua bục 1m, độ cao có tụt về 0m để thỏa mãn điều kiện hay không?"*

Phân tích cho thấy **PX4 (EKF2) tách biệt giữa độ cao địa hình (Terrain) và độ cao tuyệt đối (Absolute Altitude)**.
- Khi bay qua bục 1m, Range Finder báo giảm 1m.
- Nhưng Gia tốc kế (IMU) và Khí áp kế (Baro) báo drone không hề rơi.
- EKF2 kết luận: Có vật cản nhô lên, độ cao tuyệt đối `pos_enu_.z` của drone **vẫn được duy trì ở 1.0m**.

> [!CAUTION]
> **Kết luận tử huyệt:** 
> Dùng độ cao tuyệt đối `pos_enu_.z` làm điều kiện hạ cánh là một **thiết kế rủi ro cao**, vì hệ thống sẽ thất bại hoặc kẹt State Machine nếu chiều cao của bục đáp thay đổi mà người dùng quên cập nhật tham số `final_alt` trong YAML.

---

## 4. Giải Pháp Hiện Tại (Tạm Thời - Hotfix)
Chỉ cần chỉnh sửa tham số trong `offboard_precland_params.yaml` sao cho `final_alt` luôn lớn hơn chiều cao lớn nhất có thể của bục đáp:
- Sửa `final_alt: 0.3` (30cm). Ngưỡng kích hoạt sẽ là 0.35m.
- Sửa `abort_alt: 0.5` (50cm) để đảm bảo biên độ an toàn Point of No Return luôn lớn hơn `final_alt`.
- Khi đó, drone sẽ vào `FINAL_APPROACH` từ độ cao 35cm (khi còn lơ lửng trên không), và logic đếm chạm đất 15cm sẽ hoạt động tức thời khi bụng drone cạ vào bục 18cm.

---

## 5. Thiết Kế Tối Ưu Triệt Để (Cho đợt Refactor)

Để chấm dứt hoàn toàn sự phụ thuộc vào biến cấu hình cứng `final_alt` và miễn nhiễm với chiều cao bục đáp thực tế, chúng ta sẽ áp dụng thuật toán **Virtual Pad Altitude (Ghi nhớ cao độ bục đáp)**.

### Cơ chế hoạt động:
1. **Liên tục Cập nhật (Continuous Smoothing):**
   Trong suốt quá trình hạ độ cao, chừng nào camera còn nhìn thấy Aruco tag, hệ thống sẽ tính:
   `pad_z_ = pos_enu_.z (độ cao drone) - aruco_z (khoảng cách camera đến bục)`
   Biến này sẽ được đi qua bộ lọc **Exponential Moving Average (EMA)** để triệt tiêu nhiễu (noise) do gió giật hoặc sai số ước lượng pose.

2. **Khóa đúng thời điểm (Smart Freeze):**
   Thay vì khóa ở một độ cao ngẫu nhiên, hệ thống sẽ tự động "đóng băng" giá trị `pad_z_` **ngay khoảnh khắc trước khi camera bị mù** (khi mất tag do xuống quá thấp < 0.4m, hoặc khi marker bị cắt viền màn hình sinh ra rác). Con số cuối cùng này là kết quả đo lường chính xác nhất trong toàn bộ chuyến bay.

3. **Rơi mù an toàn tuyệt đối:**
   Ở phân đoạn cuối (dưới 0.4m), dù camera mù, ta dùng phép tính:
   `khoảng_cách_còn_lại = pos_enu_.z - pad_z_`
   Phép tính này cung cấp độ dài vật lý chính xác đến bục đáp bất chấp Baro có bị trôi vài mét hay bục cao bao nhiêu. 

> [!TIP]
> **Lợi Ích Kiến Trúc:**
> - Giải quyết dứt điểm điểm yếu "Camera mù ở phút chót".
> - Code FSM không cần biết bục cao 0.18m hay 10m.
> - Miễn nhiễm hoàn toàn với nhiễu và độ trôi của Baro (Khí áp kế) theo thời gian.
