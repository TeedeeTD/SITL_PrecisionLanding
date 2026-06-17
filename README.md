# PX4 Gimbal Precision Landing

Project này chứa ba pipeline hạ cánh chính xác cho drone `x500_gimbal` trong mô phỏng Gazebo SITL, sử dụng **MAVROS** làm giao thức kết nối điều khiển chính:

1. **Fractal ArUco landing**: Sử dụng bộ tracker C++ `aruco_fractal_tracker` với cấu trúc marker lồng nhau (nested fractal marker) tùy chỉnh có kích thước ngoài cùng 50 cm.
2. **Standard ArUco landing**: Bộ định vị Python linh hoạt (`aruco_tracker` & `aruco_precision_lander`) cho phép phát hiện các marker chuẩn từ nhiều thư viện (vd: `DICT_4X4_50`, `DICT_ARUCO_MIP_36h12`).
3. **AprilTag landing**: Bộ định vị Python (`apriltag_tracker` & `apriltag_precision_lander`) hỗ trợ phát hiện AprilTag (vd: `tag36h11`).

---

## Kiến trúc: uXRCE-DDS vs MAVROS

Trong cấu hình mô phỏng này, luồng truyền nhận thông tin được phân chia rõ ràng:
* **uXRCE-DDS Agent**: Đóng vai trò là cầu nối trực tiếp, hiệu năng cao giữa PX4 Autopilot và ROS 2 dành cho các dữ liệu telemetry nội bộ và điều khiển gimbal gốc (gimbal control topics). 
* **MAVROS**: Đóng vai trò là kênh giao tiếp điều khiển bay Offboard (local ENU setpoints, state monitoring, arming, mode change). Cả ba pipeline hạ cánh đều sử dụng MAVROS để gửi tọa độ điểm đích hạ cánh (landing target setpoints) và nhận phản hồi trạng thái từ bộ ước lượng của PX4.

---

## Cấu Trúc Thư Mục

Đặt thư mục dự án trong cây thư mục PX4 checkout:
```text
~/PX4
└── examples
    └── gimbal_simulation
```

Khi chạy mô phỏng, Gazebo load world/model từ:
```text
~/PX4/Tools/simulation/gz/worlds
~/PX4/Tools/simulation/gz/models
```

Sau khi clone hoặc chỉnh sửa world/model trong thư mục `gimbal_simulation/px4/`, đồng bộ hóa bằng `rsync`:
```bash
cd ~/PX4
rsync -a \
  examples/gimbal_simulation/px4/Tools/simulation/gz/ \
  Tools/simulation/gz/
```

---

## Yêu Cầu Hệ Thống

* **ROS 2 Humble** và **Gazebo Garden/Fortress**
* **Micro-XRCE-DDS-Agent** bản v2.4.2 (tương thích PX4)
* **MAVROS** và **Geodesy** packages
* OpenCV (`python3-opencv`) và các ROS bridges (`ros-humble-ros-gz-image`, `ros-humble-ros-gz-bridge`)

### Cài Đặt Micro-XRCE-DDS-Agent (v2.4.2)
Nếu đang có bản agent cài từ snap cũ, hãy gỡ ra trước:
```bash
sudo snap remove micro-xrce-dds-agent
```

Build agent v2.4.2 từ source để tránh các giới hạn quyền mạng của snap:
```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Khởi chạy Agent:
```bash
MicroXRCEAgent udp4 -p 8888
```

### Cài đặt ROS 2 Dependencies
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
Sử dụng marker Fractal lồng ghép đặc biệt để duy trì khả năng bám bắt mục tiêu liên tục từ độ cao 10m xuống đến mặt đất.

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
Sử dụng một marker ArUco đơn lẻ chuẩn (mặc định kích thước `0.35m`). Tích hợp bộ lọc ngưỡng kép (Double-pass Otsu) chống nhiễu và cơ chế **Low-altitude Land Commitment** (tự động chuyển sang `LAND` dưới 1.5m khi tag đi ra ngoài tầm nhìn của camera).

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
Sử dụng một AprilTag đơn lẻ (mặc định kích thước `0.35m`, loại `tag36h11` hoặc tương tự). Tích hợp bộ lọc ngưỡng kép (Double-pass Otsu) và cơ chế **Low-altitude Land Commitment**.

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
  Chọn topic `/landing/annotated_image` để xem hình ảnh từ camera gimbal vẽ đè các chỉ số FPS, trạng thái FSM hiện tại, tọa độ bám bắt mục tiêu và độ trễ.
  
* **Kiểm tra trạng thái FSM của Lander**:
  ```bash
  ros2 topic echo /lander/state
  ```

* **Kiểm tra tần số Setpoint gửi đến PX4**:
  ```bash
  ros2 topic hz /mavros/setpoint_position/local
  ```
  *(Cần đạt khoảng ~20Hz khi đang ở chế độ Offboard)*
