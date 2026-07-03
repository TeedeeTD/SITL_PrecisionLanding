# PX4 Gimbal Precision Landing

Project này chứa ba pipeline hạ cánh chính xác cho drone `x500_gimbal` trong mô phỏng Gazebo SITL, sử dụng **MAVROS** làm giao thức kết nối điều khiển chính:

1. **Fractal ArUco landing**: Sử dụng bộ tracker C++ `aruco_fractal_tracker` với cấu trúc marker lồng nhau (nested fractal marker) tùy chỉnh có kích thước ngoài cùng 50 cm.
2. **Standard ArUco landing**: Bộ định vị C++ hiệu năng cao (`aruco_tracker` trong package `aruco_fractal_tracker` & Python `aruco_precision_lander`) cho phép phát hiện các marker chuẩn.
3. **AprilTag landing**: Bộ định vị C++ hiệu năng cao (`apriltag_tracker` trong package `aruco_fractal_tracker` & Python `apriltag_precision_lander`) hỗ trợ phát hiện AprilTag.

---

## Cấu Trúc Thư Mục & Đồng Bộ Mô Phỏng

1. **Clone Dự Án**: Clone repository này vào thư mục `examples` của cây thư mục PX4 checkout:

   ```bash
   cd ~/PX4/examples
   git clone git@github.com:do010303/gimbal_simulation.git
   ```

   Cấu trúc thư mục mong đợi:
   ```text
   ~/PX4
   └── examples
       └── gimbal_simulation
   ```

2. **Đồng Bộ Hóa Mô Phỏng (Sync Worlds, Models & Textures)**:
   PX4 Gazebo sẽ load các world và model từ thư mục nội bộ của PX4. Đồng bộ hóa toàn bộ tài nguyên mô phỏng (bao gồm các file world `.sdf`, mô hình `x500_gimbal`, mô hình marker Fractal `fractal_aruco_marker`, và mô hình box `dib_box_landing_pad`) bằng lệnh `rsync`:

   ```bash
   cd ~/PX4
   rsync -a \
     examples/gimbal_simulation/px4/Tools/simulation/gz/ \
     Tools/simulation/gz/
   ```

3. **Kiểm tra các tệp tin chính**:

   ```bash
   # Kiểm tra worlds
   ls ~/PX4/Tools/simulation/gz/worlds/apriltag_landing.sdf
   ls ~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf
   ls ~/PX4/Tools/simulation/gz/worlds/fractal_aruco_landing.sdf

   # Kiểm tra mô hình và kết cấu ảnh của Fractal
   ls ~/PX4/Tools/simulation/gz/models/fractal_aruco_marker/model.sdf
   ls ~/PX4/Tools/simulation/gz/models/fractal_aruco_marker/marker.png
   ls ~/PX4/Tools/simulation/gz/models/fractal_aruco_marker/custom_fractal.yml

   # Kiểm tra mô hình box landing pad
   ls ~/PX4/Tools/simulation/gz/models/dib_box_landing_pad/model.sdf
   ```

---

## Yêu Cầu

Cần có:

- PX4 Gazebo simulation chạy được.
- PX4 `gz_x500_gimbal` chạy được.
- ROS 2 Humble.
- MAVROS cho các pipeline điều khiển hạ cánh chính xác.
- `ros_gz_image`, `cv_bridge`, `rqt_image_view`.
- ArUco C++ library có `libaruco.so.3.1`.

### Cài package ROS 2 thường dùng

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-image \
  ros-humble-ros-gz-bridge \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view \
  python3-colcon-common-extensions \
  python3-opencv

pip3 install pymavlink
```

---

## Build

`px4_msgs` cần nằm trong workspace hoặc được source từ workspace khác:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash

# Nếu chưa có px4_msgs:
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs

colcon build --symlink-install
source install/setup.bash
```

Nếu tracker thiếu `libaruco.so.3.1`, kiểm tra:

```bash
ldd install/aruco_fractal_tracker/lib/aruco_fractal_tracker/aruco_fractal_tracker | grep aruco
```

Nếu chưa resolve tới `/home/ducanh/.local/lib/libaruco.so.3.1`, rebuild tracker:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select aruco_fractal_tracker --cmake-clean-cache
source install/setup.bash
```

---

## Dọn Tiến Trình Cũ

```bash
pkill -9 -f "gz sim|px4|mavros|tracker|lander|rqt_image_view|ros_gz"
```

---

## 1. AprilTag Landing (MAVROS-based)

World AprilTag sử dụng một AprilTag đơn lẻ (mặc định kích thước `0.50m`). Node sử dụng bộ lọc ngưỡng kép (Double-pass Otsu) chống nhiễu và cơ chế **Low-altitude Land Commitment** (tự động hạ cánh dưới 1.5m khi tag đi ra ngoài camera).

* **Terminal 1: Khởi động PX4 SITL**

  ```bash
  cd ~/PX4
  PX4_GZ_WORLD=apriltag_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  ```

* **Terminal 2: Khởi động toàn bộ cụm ROS 2 Nodes (Unified Launch File)**

  ```bash
  source /opt/ros/humble/setup.bash
  source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
  ros2 launch px4_offboard apriltag_landing.launch.py target_tag_id:=0 marker_size:=0.50
  ```

---

## 2. Standard ArUco Landing (MAVROS-based)

World ArUco sử dụng một marker ArUco đơn lẻ chuẩn (mặc định kích thước `0.50m`). Node tích hợp bộ lọc Otsu threshold kép và cơ chế **Low-altitude Land Commitment**.

* **Terminal 1: Khởi động PX4 SITL**

  ```bash
  cd ~/PX4
  PX4_GZ_WORLD=aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  ```

* **Terminal 2: Khởi động toàn bộ cụm ROS 2 Nodes (Unified Launch File)**

  ```bash
  source /opt/ros/humble/setup.bash
  source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
  ros2 launch px4_offboard aruco_landing.launch.py dictionary:=DICT_4X4_50 marker_size:=0.50
  ```

---

## 3. Fractal ArUco Landing (MAVROS-based)

Pipeline định vị hạ cánh chính xác sử dụng MAVROS. Tracker C++ xuất contract `/landing/target_camera` (`dib_msgs/LandingTarget6D`) trong camera optical frame với state `LOST/SEARCHING/TRACKING`; topic pose `/aruco_fractal_tracker/poses` được giữ cho debug. Lander lọc target, bù camera offset, xoay theo yaw thân drone và điều khiển trong local ENU.

Cấu hình SITL hiện tại:

```text
box pose:     x=4.0, y=-3.5, z=0.0, yaw=0.0
marker:       0.50 m x 0.50 m, mounted on dib_box_landing_pad
camera:       1280 x 720, 30 Hz
horizontal FOV: 1.4137 rad (81°)
control loop: 30 Hz target, requirement >= 20 Hz
```
Kích thước vật lý thực tế của từng tầng:
Tầng ngoài cùng (Outer - Level 1): 50 cm (0.50 m)
Tầng giữa (Middle - Level 2): 12.5 cm (0.125 m)
Tầng trong cùng (Inner - Level 3): 3.125 cm (0.03125 m)

`dib_box_landing_pad/model.sdf`, `marker_size` trong launch và marker vật lý phải luôn dùng cùng kích thước. Detector sử dụng `custom_fractal.yml`; file này được sync sang PX4 cùng model.

* **Terminal 1: Khởi động PX4 SITL**

  ```bash
  cd ~/PX4
  PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  ```

* **Terminal 2: Chạy MAVROS một lần và giữ nguyên**

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
```

Kiểm tra MAVROS đã nối PX4:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 topic echo --once /mavros/state
```

Kỳ vọng:

```text
connected: true
```

* **Terminal 3: Khởi động bridge camera, tracker và lander**

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch px4_offboard fractal_aruco_landing.launch.py
```

`fractal_aruco_landing.launch.py` không khởi động MAVROS. Khi cần sửa/restart tracker hoặc lander, chỉ restart Terminal 3. Không restart Terminal 2, như vậy PX4 vẫn nhận heartbeat mission computer từ MAVROS liên tục.

Nếu restart MAVROS trong lúc đang bay hoặc đang giữ OFFBOARD, QGroundControl/PX4 có thể báo:

```text
Critical: Connection to mission computer lost
```

Nếu PX4 checkout không nằm tại `~/PX4`, truyền đường dẫn cấu hình marker:

```bash
ros2 launch px4_offboard fractal_aruco_landing.launch.py \
  marker_configuration:=/absolute/path/to/custom_fractal.yml
```

Controller dùng ENU cho logic hạ cánh:

```text
search_x = East
search_y = North
pos_enu / target_enu / raw_enu / sp_enu đều là ENU
```

### 3.1 Box Hybrid Landing (SITL prototype)

Pipeline thử nghiệm cho flow `box_manager + precision landing` dùng cùng PX4 SITL, MAVROS và fractal tracker, nhưng thay lander cũ bằng FSM hybrid:

```text
IDLE -> DRONE_MISSION -> PRELANDING_CHECK -> WAIT_BOX_READY
     -> SEARCH -> HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET
     -> LAND -> FLIGHT_IN_PROGRESS -> DONE
```

Trong prototype này, box thật được thay bằng `sim_box_manager`, publish `/sim_box/state`:

```text
IDLE -> PREPARING_FOR_LANDING -> WAITING_FOR_LANDING
```

Gazebo world đã có box tĩnh `dib_box_landing_pad` tại:

```text
x=4.0, y=-3.5, z=0.0, yaw=0.0
```

Trong Phase 2, vị trí này là fixture mô phỏng cho box/mission/marker. UAV nên bay tới vùng này bằng mission hoặc waypoint của box; hybrid lander chỉ bắt đầu visual refinement sau khi mission/prelanding đã hoàn tất, không dùng `search_x/search_y` để bay tới box.

Chạy PX4 SITL và MAVROS giống mục Fractal ArUco Landing ở trên. Terminal 3 đổi sang launch hybrid:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch px4_offboard box_hybrid_landing.launch.py
```

Hybrid lander không tự khởi động mission và không tự bay OFFBOARD tới box trong state `DRONE_MISSION`. Hãy setup/khởi chạy mission bằng QGroundControl hoặc luồng mission thật, rồi gửi trigger để node bắt đầu monitor mission/box:

```bash
ros2 topic pub --once /box_hybrid_landing/trigger std_msgs/msg/String "data: 'land'"
```

Trong SITL, node dùng waypoint progress hoặc khoảng cách local tới box fixture `(4.0, -3.5)` để nhận biết đã tới vùng hạ cánh. `manual_drive_alt` mặc định là `10.0m`, đóng vai trò độ cao approach/visual acquire ban đầu. Chỉ sau đó nó mới chuẩn bị gimbal, gửi `REQUEST_LANDING` tới box và chuyển sang visual guidance.

Visual guidance mặc định dùng OFFBOARD setpoint sau khi đã tới box:

```bash
ros2 launch px4_offboard box_hybrid_landing.launch.py enable_offboard_visual_servo:=true
```

Kiểm tra FSM:

```bash
ros2 topic echo /box_hybrid_landing/state
ros2 topic echo /box_hybrid_landing/box_state
ros2 topic echo /box_hybrid_landing/comms
```

Yaw alignment hiện có guard:

```bash
ros2 launch px4_offboard box_hybrid_landing.launch.py enable_yaw_setpoint:=true yaw_gate_deg:=5.0
```

Chỉ bật sau khi đã xác nhận quyền điều khiển mode/setpoint không xung đột với PX4/MAVROS mission flow. Khi bật, yaw được align tại `final_alt` trong lúc giữ XY/altitude, rồi mới trigger `AUTO.LAND`. Có thể siết `yaw_gate_deg:=3.0` khi muốn test chính xác hơn.

---

## Giám Sát và Kiểm Tra

Quy trình nghiệm thu đầy đủ cho Fractal ArUco nằm ở:

```bash
~/PX4/examples/gimbal_simulation/docs/FLIGHT_TEST.md
```

FSM hiện tại của pipeline Fractal ArUco độc lập nằm ở:

```bash
~/PX4/examples/gimbal_simulation/docs/fractal_aruco_fsm.png
```

Proposal FSM cho hướng tích hợp mission-driven với `box_manager` nằm ở:

```bash
~/PX4/examples/gimbal_simulation/docs/main_fsm.mmd
~/PX4/examples/gimbal_simulation/docs/precision_landing_fsm.mmd
```

Kế hoạch mô phỏng SITL cho box-driven hybrid landing nằm ở:

```bash
~/PX4/examples/gimbal_simulation/docs/BOX_HYBRID_SITL_PLAN.md
```

* **Xem luồng camera có telemetry HUD**:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 run rqt_image_view rqt_image_view
  ```
  Chọn topic `/landing/annotated_image` để xem hình ảnh bám bắt mục tiêu trực quan cùng thông tin telemetry (FPS, FSM State, coordinates, TVEC).

* **Kiểm tra trạng thái FSM của Lander**:
  ```bash
  ros2 topic echo /lander/state
  ```

* **Kiểm tra tần số Setpoint gửi đến PX4**:
  ```bash
  ros2 topic hz /mavros/setpoint_position/local
  ```
  *(Timer điều khiển chạy 30Hz; kết quả đo nghiệm thu cần đạt >=20Hz khi đang ở chế độ Offboard.)*

---

## Kiểm Tra Nhanh

Camera bridge:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /gimbal_camera
```

Tracker target và pose debug:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 topic hz /landing/target_camera
ros2 topic echo --once /landing/target_camera
ros2 topic hz /aruco_fractal_tracker/poses
ros2 topic echo --once /aruco_fractal_tracker/poses
```

MAVROS topics:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 topic hz /mavros/setpoint_position/local
ros2 topic echo --once /mavros/state
ros2 topic echo --once /mavros/extended_state
ros2 topic echo --once /mavros/local_position/pose
```

---

## Dấu Hiệu Thành Công

Landing thành công khi log có dạng:

```text
Marker detected
State: SEARCH -> HORIZONTAL_APPROACH
State: HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET
Final altitude reached
PX4 land detector reports landed
LANDING COMPLETE
```

Không dùng force-disarm làm tiêu chuẩn thành công khi chạy thật.

---

## Ghi Chú

- AprilTag, Standard ArUco và Fractal ArUco hiện tại đều sử dụng chung hệ thống MAVROS-based.
- Thống nhất kích thước tất cả các marker chính về `0.50m` (`marker_size:=0.50`).
- Nếu sau này đổi physical marker size trong `model.sdf`, phải đổi `marker_size` tương ứng trong file launch.
- `command 520 unsupported` là capability request MAVLink cũ từ client và không phải lệnh điều khiển landing.
- Trước một lần chạy sạch từ đầu, dừng các tiến trình PX4/MAVROS cũ để tránh giữ UDP endpoint hoặc quyền điều khiển gimbal từ phiên trước.
- Khi đang debug giữa chuyến bay hoặc đang giữ OFFBOARD, không restart MAVROS. Hãy giữ Terminal 2 chạy MAVROS riêng và chỉ restart Terminal 3 với `ros2 launch px4_offboard fractal_aruco_landing.launch.py`.
