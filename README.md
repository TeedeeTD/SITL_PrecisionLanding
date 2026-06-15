# PX4 Gimbal Precision Landing

Project này chứa hai pipeline hạ cánh chính xác cho PX4 Gazebo `x500_gimbal`:

- **AprilTag landing**: dùng OpenCV AprilTag detector trong node Python (chạy trên nền uXRCE-DDS).
- **Fractal ArUco landing**: dùng C++ `aruco_fractal_tracker` với marker `FRACTAL_5L_6`, và điều khiển hạ cánh chính xác qua **MAVROS** (hệ tọa độ ENU, hỗ trợ Gimbal Manager V2).

## Cấu Trúc

Đặt project trong PX4 checkout:

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

Kiểm tra hai world chính:

```bash
ls ~/PX4/Tools/simulation/gz/worlds/apriltag_landing.sdf
ls ~/PX4/Tools/simulation/gz/worlds/fractal_aruco_landing.sdf
ls ~/PX4/Tools/simulation/gz/models/fractal_aruco_marker/model.sdf
```

## Yêu Cầu

Cần có:

- PX4 Gazebo simulation chạy được.
- PX4 `gz_x500_gimbal` chạy được.
- ROS 2 Humble.
- `MicroXRCEAgent`.
- `ros_gz_image`, `cv_bridge`, `rqt_image_view`.
- ArUco C++ library có `libaruco.so.3.1`.

### Cài Micro-XRCE-DDS-Agent 2.4.2

Nếu đang dùng bản snap cũ, gỡ trước:

```bash
sudo snap remove micro-xrce-dds-agent
```

#### Option A: Build from Source (Recommended)

Building from source is recommended as it avoids sandbox/network restrictions and works reliably with localhost-only configurations:

```bash
# Clone branch v2.4.2 để tương thích với PX4
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

#### Option B: Build via ROS 2 Workspace (Colcon)

Bypass bằng cách build trong ROS 2 workspace:

```bash
mkdir -p ~/px4_ros_uxrce_dds_ws/src
cd ~/px4_ros_uxrce_dds_ws/src
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

cd ~/px4_ros_uxrce_dds_ws
source /opt/ros/humble/setup.bash
colcon build
```

Khi chạy agent với Option B, source workspace trước:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros_uxrce_dds_ws/install/local_setup.bash
MicroXRCEAgent udp4 -p 8888
```
#### Option C: Install via Snap (Alternative)

```bash

sudo snap install micro-xrce-dds-agent --classic

```

Once installed, you can start the agent using:

```bash

MicroXRCEAgent udp4 -p 8888

```

or:

```bash

micro-xrce-dds-agent udp4 -p 8888
```

### Cài package ROS 2 thường dùng

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-image \
  ros-humble-ros-gz-bridge \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view \
  python3-colcon-common-extensions \
  python3-opencv

pip3 install pymavlink
```

Nếu dùng `ROS_LOCALHOST_ONLY=1`, trong PX4 console chạy một lần:

```bash
param set UXRCE_DDS_PTCFG 1
```

Sau đó restart PX4 SITL.

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

## Dọn Tiến Trình Cũ

```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_image|ros_gz_bridge|aruco_fractal_tracker|fractal_aruco_precision_lander|apriltag_precision_lander|rqt_image_view"
```

## AprilTag Landing

World AprilTag có bốn target `tag25h9`:

```text
tag 0: x= 3.0, y= 2.0
tag 1: x= 3.0, y=-2.0
tag 2: x=-3.0, y= 2.0
tag 3: x=-3.0, y=-2.0
```

Terminal 1:

```bash
cd ~/PX4
PX4_GZ_WORLD=apriltag_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros_uxrce_dds_ws/install/local_setup.bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_image image_bridge \
  "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image" \
  --ros-args \
  -r "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera"
```

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash

ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=0
```

## Fractal ArUco Landing (MAVROS-based)

Pipeline định vị hạ cánh chính xác sử dụng MAVROS thay cho uXRCE-DDS. Pipeline này tự động xoay hệ tọa độ theo góc quay thực tế của drone (`camera_yaw_frame:=body`) và tương thích với Gimbal Manager Protocol V2 của PX4 v1.15+.

Trong chế độ này, mô hình marker được đổi sang kích thước thực tế **0.30m x 0.30m**, do đó tracker và world được cấu hình với kích thước `0.30`.

### Terminal 1: Khởi động PX4 SITL
```bash
cd ~/PX4
PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

### Terminal 2: Khởi động MAVROS
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
```

### Terminal 3: Khởi động Gazebo Image & Clock Bridge
```bash
source /opt/ros/humble/setup.bash

# Chạy Image Bridge (ở background)
ros2 run ros_gz_image image_bridge \
  "/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image" \
  --ros-args \
  -r "/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera" &

# Chạy Clock Bridge (để đồng bộ time)
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Terminal 4: Khởi động C++ Tracker Node (với use_sim_time:=true)
```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 run aruco_fractal_tracker aruco_fractal_tracker --ros-args \
  -p marker_configuration:=FRACTAL_5L_6 \
  -p marker_size:=0.30 \
  -p show_latency_overlay:=true \
  -p latency_warn_ms:=100.0 \
  -p use_sim_time:=true \
  -r image_input_topic:=/gimbal_camera \
  -r camera_info_topic:=/gimbal_camera/camera_info \
  -r image_output_topic:=/landing/annotated_image \
  -r poses_output_topic:=/aruco_fractal_tracker/poses
```

### Terminal 5: Khởi động MAVROS-based Lander Node (với use_sim_time:=true)
```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 run px4_offboard fractal_aruco_precision_lander --ros-args \
  -p search_frame:=enu \
  -p search_x:=3.0 \
  -p search_y:=2.0 \
  -p cruise_alt:=5.0 \
  -p camera_yaw_frame:=body \
  -p camera_x_to_body_east_sign:=1.0 \
  -p camera_y_to_body_north_sign:=-1.0 \
  -p use_sim_time:=true \
  -p pose_topic:=/aruco_fractal_tracker/poses
```

Controller dùng ENU cho logic hạ cánh:

```text
search_x = East
search_y = North
pos_enu / target_enu / raw_enu / sp_enu đều là ENU
```

### Xem và Giám Sát:
- **Ảnh Annotated camera & Latency:** Mở `ros2 run rqt_image_view rqt_image_view` và chọn topic `/landing/annotated_image`.
- **Tần số setpoint:** `ros2 topic hz /mavros/setpoint_position/local` (phải đạt ~20 Hz trong khi bay Offboard).
- **Trạng thái MAVROS:** `ros2 topic echo --once /mavros/state`

## Xem Camera

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Chọn:

```text
/landing/annotated_image
```

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

PX4 ROS 2 topics:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 topic echo --once /fmu/out/vehicle_status_v4
ros2 topic echo --once /fmu/out/vehicle_local_position_v1
```

## Dấu Hiệu Thành Công

Fractal landing thành công khi log có dạng:

```text
Fractal marker detected
State: SEARCH -> HORIZONTAL_APPROACH
State: HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET
Fractal final altitude reached
PX4 land detector reports landed
LANDING COMPLETE
```

Không dùng force-disarm làm tiêu chuẩn thành công khi chạy thật.

## Ghi Chú

- AprilTag vẫn là pipeline riêng và giữ nguyên.
- Fractal ArUco là pipeline ArUco chính của project.
- Với `fractal_aruco_landing`, dùng `marker_size:=1.0`.
- Nếu sau này đổi physical marker size trong `fractal_aruco_marker/model.sdf`, phải đổi `marker_size` tương ứng.
