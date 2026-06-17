# Fractal ArUco Flight Test

File này gom toàn bộ quy trình kiểm tra một chuyến bay SITL cho pipeline Fractal ArUco MAVROS. Một lần test chỉ được coi là đạt khi có đủ evidence ở các mục pass/fail bên dưới.

## Mục Tiêu

- Drone takeoff, gimbal nhìn xuống, bay tới vùng search.
- Tracker phát hiện marker Fractal ở `1280x720`, FOV `1.4137 rad`, marker ngoài `0.50 m`.
- Lander đi đúng FSM: `SEARCH -> HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET -> LAND -> DONE`.
- Setpoint descent giảm đều, không bị nhảy ngược về cruise/search altitude khi tracker mất vài frame.
- PX4 land detector báo landed và node disarm sau khi chạm đất.

## Chuẩn Bị

```bash
cd ~/PX4
rsync -a examples/gimbal_simulation/px4/Tools/simulation/gz/ Tools/simulation/gz/

cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select aruco_fractal_tracker px4_offboard
```

Dọn phiên cũ trước khi chạy:

```bash
pkill -9 -f "gz sim|px4|mavros|tracker|lander|rqt_image_view|ros_gz"
```

## Chạy Test

Terminal 1:

```bash
cd ~/PX4
PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch px4_offboard fractal_aruco_landing.launch.py
```

## Record Evidence

Chạy trong Terminal 3 ngay sau khi launch:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
mkdir -p ~/PX4/examples/gimbal_simulation/flight_tests
ros2 bag record -o ~/PX4/examples/gimbal_simulation/flight_tests/fractal_$(date +%Y%m%d_%H%M%S) \
  /lander/state \
  /landing/target_camera \
  /mavros/state \
  /mavros/extended_state \
  /mavros/local_position/pose \
  /mavros/setpoint_position/local \
  /mavros/landing_target/raw
```

Nếu cần xem trực tiếp trong lúc chạy:

```bash
ros2 topic echo /lander/state
ros2 topic hz /mavros/setpoint_position/local
ros2 topic echo /mavros/setpoint_position/local
ros2 topic echo /landing/target_camera
```

## Tiêu Chí Pass

Một run đạt khi thỏa tất cả điều kiện:

- FSM có đủ chuỗi: `SEARCH -> HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET -> LAND -> DONE`.
- Không có vòng lặp lặp lại nhiều lần kiểu `DESCEND_OVER_TARGET -> TARGET_LOST -> SEARCH` trong lúc đang ở cao độ lớn.
- `/mavros/setpoint_position/local.pose.position.z` giảm đều trong `DESCEND_OVER_TARGET`.
- Setpoint `z` không nhảy ngược về `CRUISE_ALT` trong descent khi `/landing/target_camera` có vài frame `SEARCHING/LOST`.
- Sai số lateral trong log `Fractal visual descent` nằm trong descent gate; giá trị tốt hiện tại thường dưới `0.20 m`.
- Cuối run có log:

```text
Fractal final altitude reached
PX4 land detector reports landed - disarming
LANDING COMPLETE
State: LAND -> DONE
```

- `/mavros/extended_state.landed_state` chuyển về trạng thái landed trước khi node báo `DONE`.
- `/mavros/setpoint_position/local` đạt tối thiểu `20 Hz`; target hiện tại là `30 Hz`.

## Tiêu Chí Fail

Run phải coi là fail nếu gặp một trong các dấu hiệu:

- FSM không tới `DONE`.
- Lander bị kẹt ở `SEARCH`, `HORIZONTAL_APPROACH`, `DESCEND_OVER_TARGET`, hoặc `TARGET_LOST`.
- Trong descent, setpoint `z` tăng mạnh hoặc quay về `CRUISE_ALT`.
- Tracker có detection nhưng lander không nhận pose accepted trong nhiều giây.
- PX4 land detector không báo landed sau khi đã xuống gần mặt đất.
- Cần `force_disarm` mới kết thúc. Force disarm chỉ dùng để recover mô phỏng, không dùng làm tiêu chuẩn pass.
- MAVROS mất kết nối hoặc rớt khỏi `OFFBOARD` trong nhiệm vụ.

## Cách Đọc Log Nhanh
Tìm log:
```bash
ls -lh ~/PX4/examples/gimbal_simulation/flight_tests
```
Phân tích tự động từ bag:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
python3 ~/PX4/examples/gimbal_simulation/scripts/analyze_fractal_flight.py \
  ~/PX4/examples/gimbal_simulation/flight_tests/fractal_YYYYMMDD_HHMMSS
```

Tìm state transition:

```bash
rg "State:|LANDING COMPLETE|PX4 land detector|final altitude|target lost" <log_file>
```

Dấu hiệu run tốt:

```text
State: SEARCH -> HORIZONTAL_APPROACH
State: HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET
Fractal visual descent (descending): ... sp_enu=[..., ..., 9.xx]
Fractal visual descent (descending): ... sp_enu=[..., ..., 5.xx]
Fractal visual descent (descending): ... sp_enu=[..., ..., 0.30]
State: DESCEND_OVER_TARGET -> LAND
LANDING COMPLETE
State: LAND -> DONE
```

Dấu hiệu run xấu:

```text
State: DESCEND_OVER_TARGET -> TARGET_LOST
Fractal target lost above commit altitude - holding search pose
State: TARGET_LOST -> SEARCH
```

Một vài dòng `No fractal marker yet` hoặc vài frame `/landing/target_camera state=SEARCHING/LOST` là chấp nhận được nếu lander vẫn tiếp tục descent ổn định.

## Warning Được Chấp Nhận Trong SITL

- `command 520 unsupported`: capability request MAVLink cũ, không phải lỗi landing.
- `PositionTargetGlobal failed because no origin`: MAVROS global target plugin chưa có global origin; pipeline đang dùng local setpoint.
- `Unexpected command 176, result 0`: thường xuất hiện khi set mode/land command trả result thành công hoặc không chặn nhiệm vụ.

## Ghi Chú Frame ENU

Trong pipeline hiện tại, `pos_enu`, `target_enu`, `sp_enu`, `raw_enu` và `filt_enu` trong log của lander là local ENU/map frame của MAVROS/UAV:

- `East`: trục X local/map.
- `North`: trục Y local/map.
- `Up`: trục Z local/map.

Đây không phải ENU gắn với camera gimbal. Tracker xuất `tvec` trong `camera_link`; lander lấy thành phần ảnh/camera, bù offset camera so với thân UAV, xoay theo yaw của UAV rồi mới đưa về local ENU. Khi publish `/mavros/landing_target/raw`, node đổi local ENU sang `LOCAL_NED` cho MAVROS/PX4.

## Kết Luận Report

Sau mỗi run, ghi ngắn:

```text
Run ID:
Date:
Result: PASS/FAIL
FSM sequence:
Minimum altitude:
Final landed_state:
Max visual descent error:
Setpoint rate:
Notes:
```

Khuyến nghị nghiệm thu: chạy tối thiểu 3 run PASS liên tiếp sau mỗi lần đổi tracker, lander, marker size, camera FOV hoặc gimbal control.
