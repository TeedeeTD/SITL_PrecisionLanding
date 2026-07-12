# Kiến Trúc Tái Cấu Trúc (Refactoring Architecture)
**Dự án:** ROS 2 Offboard Precision Landing Controller

---

## 1. Vấn Đề Hiện Tại (Motivation)

Bộ điều khiển hiện tại (`offboard_precland_controller.cpp`) đang hoạt động tốt nhưng phình to lên tới ~1500 dòng code. Việc gộp chung tất cả vào một file duy nhất vi phạm nghiêm trọng nguyên tắc **Single Responsibility (Đơn nhiệm)**.

Trong firmware PX4 gốc, 1500 dòng logic này được phân bổ cho 4-5 module khác nhau (như `precland`, `land_detector`, `Commander`, `matrix`...). Vì chúng ta đang viết một Offboard Node độc lập, chúng ta buộc phải "gánh" toàn bộ các chức năng đó. Thống kê hiện tại cho thấy:

| Phần | Dòng | Tỷ lệ |
|---|---|---|
| Constructor (khai báo + đọc param) | 9–239 (~230) | 15% |
| ROS callbacks (`on_pos`, `on_state`, `on_target`...) | 243–417 (~175) | 12% |
| MAVROS service helpers (`set_mode`, `disarm`, `send_command`...) | 418–547 (~130) | 9% |
| Math/frame utils (quaternion, yaw, camera→ENU, history lookup) | 548–835 (~290) | 19% |
| FSM glue (`transition`, `control_loop`, `can_transition`) | 837–1048 (~210) | 14% |
| 9 state handlers (`st_*`) | 1049–1490 (~440) | 30% |

→ Chỉ ~30% là "logic state machine" thật sự tương đương `precland.cpp`. Phần còn lại (70%) là hạ tầng — đây chính là phần nên tách ra.

Việc không chia tách code gây ra 3 hậu quả:
1. **Khó bảo trì:** Lỗi logic nhỏ (như vụ kẹt state gây delay 5s disarm) bị chôn vùi trong hàng ngàn dòng code.
2. **Khó kiểm thử (Unit Test):** Không thể test riêng lẻ thuật toán cổng động (Dynamic Gates) hay thuật toán căn góc (Yaw Lock) mà không khởi tạo cả một ROS 2 Node khổng lồ.
3. **Khó mở rộng:** Việc thêm tính năng mới (như Virtual Pad Altitude) sẽ khiến file phình to quá mức kiểm soát.

---

## 2. Thiết Kế Phân Lớp Đề Xuất (Architecture Design)

Kiến trúc mới sẽ chia cắt Node khổng lồ thành **3 Lớp (Layers)** với các Module nhỏ gọn (dưới 200 dòng/module).

### Sơ đồ Cấu trúc Thư mục (Directory Structure)
Dưới đây là cấu trúc thư mục mô phỏng đúng cách PX4 tách module:

```text
precision_landing/
├── include/precision_landing/
│   ├── offboard_precland_controller.hpp   ← chỉ còn FSM glue + state (~150 dòng)
│   ├── mavros_link.hpp                    ← toàn bộ giao tiếp MAVROS
│   ├── frame_utils.hpp                    ← quaternion/yaw/camera-to-ENU (pure functions)
│   ├── descent_profile.hpp                ← các hàm get_align_r/get_descent_r/gate/gain
│   ├── yaw_lock_manager.hpp               ← buffer + circular mean + 2-stage lock
│   ├── target_tracker.hpp                 ← on_target logic, filtering, reject gate
│   └── precland_params.hpp                ← struct param
├── src/
│   ├── offboard_precland_controller.cpp
│   ├── states/
│   │   ├── state_idle.cpp
│   │   ├── state_start.cpp
│   │   ├── state_horizontal_approach.cpp
│   │   ├── state_descend_above_target.cpp
│   │   ├── state_final_approach.cpp
│   │   ├── state_search.cpp
│   │   ├── state_target_lost.cpp
│   │   └── state_fallback.cpp
│   ├── mavros_link.cpp
│   ├── frame_utils.cpp
│   ├── descent_profile.cpp
│   ├── yaw_lock_manager.cpp
│   └── target_tracker.cpp
```

### A. Lớp Hạ Tầng (Infrastructure Layer)
Chịu trách nhiệm giao tiếp với hệ thống bên ngoài (ROS 2, MAVROS, PX4).
* **`MavrosLink`**:
  - Đóng gói toàn bộ Pub/Sub/Services của MAVROS.
  - Cung cấp các API sạch: `disarm()`, `set_mode()`, `send_command()`.
  - Toàn bộ cơ chế xử lý lỗi mạng (Retry MAVLink, fallback ACK) nằm gọn ở đây.
* **`PrecLandParams`**:
  - Thay thế 150 dòng boilerplate khai báo thông số bằng một Struct nạp tham số tự động.
* **`FrameUtils`**:
  - Chứa các hàm toán học thuần túy (Pure Functions): Biến đổi Quaternion, chuyển đổi hệ tọa độ Camera sang ENU.

### B. Lớp Dữ Liệu & Thuật Toán (Data & Algorithms)
Chịu trách nhiệm lọc, xử lý tín hiệu và nội suy. Hoàn toàn độc lập với ROS 2 (dễ dàng Unit Test).
* **`TargetTracker`**:
  - Lọc nhiễu tọa độ Aruco marker.
  - **Tích hợp "Virtual Pad Altitude"**: Liên tục tính toán `pos_enu.z - aruco_z` và tự động đóng băng giá trị (freeze) khi camera bị mù hoặc chạm viền (edge outlier). Cung cấp hàm `get_distance_to_pad()` chuẩn xác tuyệt đối.
* **`DescentProfile`**:
  - Chứa toàn bộ các hàm toán học nội suy theo độ cao: `get_align_radius()`, `get_descent_radius()`, `get_servo_gain()`, `get_alpha()`.
* **`YawLockManager`**:
  - Quản lý logic tính trung bình góc xoay, gom đủ số lượng samples (buffer) và chia làm 2 giai đoạn (Phase 1 ở 7m, Phase 2 ở 3m).

### C. Lớp Điều Khiển & Trạng Thái (Control & State Layer)
* **`OffboardPreclandController` (Context)**:
  - Lõi của ROS 2 Node nay chỉ đóng vai trò Trạm Trung Chuyển (Context). Nó nắm giữ instance của các class trên và cung cấp dữ liệu cho State Machine.
* **`PrecLandStateHandlers` (Strategy Pattern)**:
  - Chia 9 trạng thái hạ cánh hiện tại (`IDLE`, `START`, `SEARCH`, `DESCENT`, `FINAL_APPROACH`...) thành 9 struct/class riêng biệt.
  - Mỗi class bắt buộc triển khai hàm `tick(Context& ctx)` và `on_enter()`. Logic chuyển state sẽ trong sáng như PX4 gốc.

---

## 3. Lộ Trình Triển Khai (Refactoring Roadmap)

Việc đập đi xây lại 1500 dòng code có rủi ro sinh ra bug hồi quy (regression bugs). Do đó, lộ trình sẽ được chia thành **4 Giai Đoạn (Phases)**. Sau mỗi Phase đều có thể biên dịch (compile) và bay thử trên SITL để đảm bảo code vẫn chạy đúng.

> [!IMPORTANT]
> **Phase 1: Làm sạch Hạ Tầng (An toàn nhất)**
> - Tạo `precland_params.hpp` và cấu trúc lại hàm khởi tạo (Constructor).
> - Tạo `mavros_link.hpp/.cpp`, di chuyển toàn bộ logic kết nối PX4 ra khỏi controller chính.

> [!NOTE]
> **Phase 2: Cô lập Thuật Toán Thuần (Toán học)**
> - Tách `frame_utils` và `descent_profile`.
> - Bước này cực kỳ an toàn vì đây là các hàm toán học không trạng thái (stateless), chỉ copy-paste và thay đổi cú pháp gọi hàm.

> [!WARNING]
> **Phase 3: Tích hợp Tính năng Mới (TargetTracker & YawLock)**
> - Xây dựng `TargetTracker` và áp dụng thuật toán **Virtual Pad Altitude**.
> - Đưa logic đệm góc Yaw sang `YawLockManager`.
> - *Rủi ro trung bình*: Logic tính khoảng cách đáp sẽ thay đổi để thông minh hơn.

> [!CAUTION]
> **Phase 4: Cấu trúc lại State Machine (Thay Máu Toàn Bộ)**
> - Tạo base class `State` và tách 9 state handlers ra 9 file.
> - Code của controller chính sẽ giảm từ 1500 dòng xuống còn ~200 dòng.
> - *Rủi ro cao*: Dễ nhầm lẫn luồng đi của dữ liệu, cần test SITL kỹ lưỡng nhất ở giai đoạn này.

---

## 4. Kết Quả Ước Tính Sau Tái Cấu Trúc

Sau khi hoàn thành đợt refactor, tổng lượng logic gần như được giữ nguyên (~1400 - 1500 dòng), nhưng không có file nào vượt quá 200 dòng, đúng với tinh thần Single Responsibility:

| File | Dòng ước tính |
|---|---|
| `offboard_precland_controller.cpp` (glue + dispatch) | ~150–200 |
| `mavros_link.cpp` | ~180 |
| `frame_utils.cpp` | ~150 |
| `descent_profile.cpp` | ~100 |
| `yaw_lock_manager.cpp` | ~80 |
| `target_tracker.cpp` | ~120 |
| 8 file `state_*.cpp` | ~50–100 mỗi file (~600 tổng) |
| `precland_params.hpp` | ~40 |

---
*Tài liệu này đóng vai trò là kim chỉ nam cho các quyết định thiết kế trong suốt quá trình refactor hệ thống Precision Landing.*
