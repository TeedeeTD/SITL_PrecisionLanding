# PX4 Gimbal Precision Landing

Project này chứa ba pipeline hạ cánh chính xác cho drone `x500_gimbal` trong mô phỏng Gazebo SITL, sử dụng **MAVROS** làm giao thức kết nối điều khiển chính:

1. **Fractal ArUco landing**: Sử dụng bộ tracker C++ `aruco_fractal_tracker` với cấu trúc marker lồng nhau (nested fractal marker) tùy chỉnh có kích thước ngoài cùng 50 cm.
2. **Standard ArUco landing**: Bộ định vị Python linh hoạt (`aruco_tracker` & `aruco_precision_lander`) cho phép phát hiện các marker chuẩn từ nhiều thư viện (vd: `DICT_4X4_50`, `DICT_ARUCO_MIP_36h12`).
3. **AprilTag landing**: Bộ định vị Python (`apriltag_tracker` & `apriltag_precision_lander`) hỗ trợ phát hiện AprilTag (vd: `tag36h11`).

---

## Cấu Trúc Thư Mục & Đồng Bộ Mô Phỏng

Đặt thư mục dự án trong cây thư mục PX4 checkout:

```text
~/PX4
└── examples
    └── gimbal_simulation
```

PX4 Gazebo load world/model từ:

```text
~/PX4/Tools/simulation/gz/worlds
~/PX4/Tools/simulation/gz/models
```

Sau khi clone hoặc sửa world/model, sync overlay:

```bash
cd ~/PX4
rsync -a \
  examples/gimbal_simulation/px4/Tools/simulation/gz/ \
  Tools/simulation/gz/
```

Kiểm tra các world chính:

```bash
ls ~/PX4/Tools/simulation/gz/worlds/apriltag_landing.sdf
ls ~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf
ls ~/PX4/Tools/simulation/gz/worlds/fractal_aruco_landing.sdf
ls ~/PX4/Tools/simulation/gz/models/fractal_aruco_marker/model.sdf
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

Pipeline định vị hạ cánh chính xác sử dụng MAVROS. Tracker C++ xuất pose trong camera optical frame; lander lọc pose, bù camera offset, xoay theo yaw thân drone và điều khiển trong local ENU.

Cấu hình SITL hiện tại:

```text
marker:       0.50 m x 0.50 m
camera:       1280 x 720, 30 Hz
horizontal FOV: 1.2 rad
control loop: 20 Hz
```

`model.sdf`, `marker_size` trong launch và marker vật lý phải luôn dùng cùng kích thước. Detector sử dụng `custom_fractal.yml`; file này được sync sang PX4 cùng model.

* **Terminal 1: Khởi động PX4 SITL**

  ```bash
  cd ~/PX4
  PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  ```

* **Terminal 2: Khởi động toàn bộ cụm ROS 2 Nodes (Unified Launch File)**

  ```bash
  source /opt/ros/humble/setup.bash
  source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
  ros2 launch px4_offboard fractal_aruco_landing.launch.py
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

---

## Giám Sát và Kiểm Tra

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
  *(Cần đạt khoảng ~20Hz khi đang ở chế độ Offboard)*

---

## Kiểm Tra Nhanh

Camera bridge:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /gimbal_camera
```

Tracker pose:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
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
- Trước mỗi lần chạy lại, dừng các tiến trình PX4/MAVROS cũ để tránh giữ UDP endpoint hoặc quyền điều khiển gimbal từ phiên trước.
