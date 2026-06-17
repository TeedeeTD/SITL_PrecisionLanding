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

Đồng bộ hóa các tài nguyên mô phỏng (worlds và models) sang cây thư mục PX4 chính bằng `rsync`:
```bash
cd ~/PX4
rsync -a \
  examples/gimbal_simulation/px4/Tools/simulation/gz/ \
  Tools/simulation/gz/
```

---

## Yêu Cầu Hệ Thống

Cài đặt các thư viện và package ROS 2 cần thiết:
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

## Biên Dịch Không Gian Làm Việc (ROS 2 Workspace)

Di chuyển vào workspace của dự án và build các package:
```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash

# Clone px4_msgs nếu chưa có
if [ ! -d "src/px4_msgs" ]; then
  git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
fi

colcon build --symlink-install
source install/setup.bash
```

---

## Hướng Dẫn Chạy Mô Phỏng & Hạ Cánh

Trước khi chạy phiên mô phỏng mới, hãy dọn dẹp các tiến trình cũ để tránh xung đột cổng:
```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_image|ros_gz_bridge|aruco_fractal_tracker|fractal_aruco_precision_lander|apriltag_precision_lander|rqt_image_view"
```

### 1. Mô phỏng Fractal ArUco Landing
* **Terminal 1: Khởi động Gazebo & PX4**
  ```bash
  cd ~/PX4
  PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  ```

* **Terminal 2: Chạy MAVROS & Landing Nodes**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
  ros2 launch px4_offboard fractal_aruco_landing.launch.py
  ```

---

### 2. Mô phỏng Standard ArUco Landing
* **Terminal 1: Khởi động Gazebo & PX4**
  ```bash
  cd ~/PX4
  PX4_GZ_WORLD=aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  ```

* **Terminal 2: Chạy MAVROS & Landing Nodes**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
  ros2 launch px4_offboard aruco_landing.launch.py dictionary:=DICT_4X4_50 marker_size:=0.35
  ```

---

### 3. Mô phỏng AprilTag Landing
* **Terminal 1: Khởi động Gazebo & PX4**
  ```bash
  cd ~/PX4
  PX4_GZ_WORLD=apriltag_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  ```

* **Terminal 2: Chạy MAVROS & Landing Nodes**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
  ros2 launch px4_offboard apriltag_landing.launch.py target_tag_id:=0 marker_size:=0.35
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
