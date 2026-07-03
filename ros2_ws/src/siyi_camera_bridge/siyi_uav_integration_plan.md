# Kế Hoạch Tích Hợp SIYI Camera Bridge Lên UAV

Tài liệu này liệt kê các file/package cần copy lên UAV `172.20.50.40`, các dependency cần cài, các tham số cần sửa, và thứ tự kiểm tra để tích hợp dần camera SIYI với pipeline detect marker.

Mục tiêu giai đoạn đầu:

```text
SIYI RTSP stream
→ ROS 2 Image + CameraInfo
→ Fractal ArUco detector
→ /siyi/fractal_debug
→ /siyi/fractal_pose
→ /siyi/landing_target
```

## 1. Các Thư Mục Cần Copy Lên UAV

Không chỉ copy riêng `siyi_camera_bridge`. Package này cần thêm tracker và custom message.

Copy các thư mục sau:

```text
examples/gimbal_simulation/ros2_ws/src/siyi_camera_bridge
examples/gimbal_simulation/ros2_ws/src/aruco_fractal_tracker
examples/gimbal_simulation/ros2_ws/src/dib_msgs
```

Copy thêm file cấu hình marker fractal:

```text
examples/gimbal_simulation/px4/Tools/simulation/gz/models/fractal_aruco_marker/custom_fractal.yml
```

Nếu cần in/kiểm tra marker, copy thêm:

```text
examples/gimbal_simulation/px4/Tools/simulation/gz/models/fractal_aruco_marker/marker.png
```

Cấu trúc khuyến nghị trên UAV:

```text
~/siyi_ws/
└── src/
    ├── siyi_camera_bridge/
    ├── aruco_fractal_tracker/
    └── dib_msgs/

~/siyi_ws/config/
└── custom_fractal.yml
```

Ví dụ copy từ máy development sang UAV:

```bash
ssh jb@172.20.50.40 'mkdir -p ~/siyi_ws/src ~/siyi_ws/config'

scp -r examples/gimbal_simulation/ros2_ws/src/siyi_camera_bridge \
  jb@172.20.50.40:~/siyi_ws/src/

scp -r examples/gimbal_simulation/ros2_ws/src/aruco_fractal_tracker \
  jb@172.20.50.40:~/siyi_ws/src/

scp -r examples/gimbal_simulation/ros2_ws/src/dib_msgs \
  jb@172.20.50.40:~/siyi_ws/src/

scp examples/gimbal_simulation/px4/Tools/simulation/gz/models/fractal_aruco_marker/custom_fractal.yml \
  jb@172.20.50.40:~/siyi_ws/config/
```

## 2. Dependency Cần Cài Trên UAV

Trên UAV `172.20.50.40`:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  python3-opencv \
  python3-colcon-common-extensions
```

Nếu build tracker báo thiếu thư viện ArUco, cần cài hoặc build `libaruco` đúng version mà package `aruco_fractal_tracker` đang dùng.

Trong repo hiện tại, tracker cần ArUco C++ có CMake package:

```text
arucoConfig.cmake
libaruco.so
```

Trên máy development, thư viện này đang nằm dạng tương tự:

```text
~/.local/share/aruco/arucoConfig.cmake
~/.local/lib/libaruco.so.3.1
```

## 3. Build Workspace Trên UAV

```bash
cd ~/siyi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dib_msgs aruco_fractal_tracker siyi_camera_bridge
source install/setup.bash
```

Kiểm tra package đã thấy trong ROS 2:

```bash
ros2 pkg list | grep siyi_camera_bridge
ros2 pkg list | grep aruco_fractal_tracker
ros2 pkg list | grep dib_msgs
```

### 3.1 Sửa Lỗi Thiếu `arucoConfig.cmake`

Nếu build báo lỗi:

```text
Could not find a package configuration file provided by "aruco"
arucoConfig.cmake
aruco-config.cmake
```

thì UAV chưa có ArUco C++ library mà `aruco_fractal_tracker` cần.

Cách kiểm tra trên UAV:

```bash
find ~/.local /usr/local /usr -name 'arucoConfig.cmake' -o -name 'aruco-config.cmake' 2>/dev/null
find ~/.local /usr/local /usr -name 'libaruco.so*' 2>/dev/null
```

Nếu tìm thấy, ví dụ:

```text
/home/jb/.local/share/aruco/arucoConfig.cmake
```

thì build lại bằng:

```bash
cd ~/ducanh_ws/siyi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-select aruco_fractal_tracker \
  --cmake-args -Daruco_DIR=/home/jb/.local/share/aruco
```

Nếu có warning package `dib_msgs` đã tồn tại ở underlay như:

```text
Some selected packages are already built in one or more underlay workspaces
```

thì có hai hướng:

```bash
# Hướng an toàn: không source underlay workspace đang có dib_msgs trước khi build
source /opt/ros/humble/setup.bash

# Hoặc nếu hiểu rủi ro và muốn override dib_msgs trong overlay hiện tại:
colcon build --symlink-install \
  --packages-select dib_msgs aruco_fractal_tracker siyi_camera_bridge \
  --allow-overriding dib_msgs \
  --cmake-args -Daruco_DIR=/home/jb/.local/share/aruco
```

Nếu chưa có ArUco C++ trên UAV, cần build/install ArUco 3.1.x. Sau khi install,
kiểm tra phải có:

```text
~/.local/share/aruco/arucoConfig.cmake
~/.local/lib/libaruco.so
```

Trong repo hiện đã có source ArUco 3.1.12:

```text
examples/gimbal_simulation/aruco_build/aruco.zip
examples/gimbal_simulation/aruco_build/aruco-3.1.12/
```

Copy source lên UAV:

```bash
ssh jb@172.20.50.40 'mkdir -p ~/ducanh_ws/third_party'

scp examples/gimbal_simulation/aruco_build/aruco.zip \
  jb@172.20.50.40:~/ducanh_ws/third_party/
```

Build và install ArUco trên UAV:

```bash
cd ~/ducanh_ws/third_party
unzip aruco.zip

cd aruco-3.1.12
rm -rf build
mkdir build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local \
  -DBUILD_UTILS=OFF \
  -DBUILD_GLSAMPLES=OFF \
  -DINSTALL_DOC=OFF

cmake --build . -j$(nproc)
cmake --install .
```

Kiểm tra sau khi install:

```bash
ls ~/.local/share/aruco/arucoConfig.cmake
ls ~/.local/lib/libaruco.so*
```

Sau đó export đường dẫn khi build:

```bash
export CMAKE_PREFIX_PATH=$HOME/.local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH

cd ~/ducanh_ws/siyi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select dib_msgs aruco_fractal_tracker siyi_camera_bridge \
  --allow-overriding dib_msgs \
  --cmake-args -DCMAKE_PREFIX_PATH=$HOME/.local
source install/setup.bash
```

## 4. Các File Cần Sửa Hoặc Cần Tham Số Hóa

### 4.1 `real_fractal_detect.launch.py`

File:

```text
~/siyi_ws/src/siyi_camera_bridge/launch/real_fractal_detect.launch.py
```

Hiện file này đang có default:

```text
rtsp_url:=rtsp://192.168.168.14:8554/main.264
marker_configuration:=~/PX4/examples/gimbal_simulation/.../custom_fractal.yml
marker_size:=0.50
fcu_url:=udp://:14540@127.0.0.1:14580
image_width:=1280
image_height:=720
flip_180:=true
```

Khi chạy trên UAV, cần đổi bằng launch arguments hoặc sửa default.

Khuyến nghị: không hard-code vội, truyền khi chạy:

```bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=false \
  rtsp_url:=rtsp://<SIYI_IP>:8554/main.264 \
  marker_configuration:=/home/jb/siyi_ws/config/custom_fractal.yml \
  marker_size:=0.50 \
  flip_180:=true
```

Các tham số cần kiểm tra/sửa:

```text
rtsp_url              IP thật của SIYI camera.
flip_180              true/false tùy camera có bị lắp ngược hay không.
marker_configuration  Đường dẫn custom_fractal.yml trên UAV.
marker_size           Kích thước thật cạnh ngoài marker, đơn vị mét.
enable_mavros         false nếu MAVROS đã chạy riêng hoặc chỉ test camera.
fcu_url               Cổng kết nối MAVROS nếu muốn launch tự chạy MAVROS.
```

### 4.2 Thêm Launch Arguments Cho Camera Intrinsics

File node đã hỗ trợ các parameter này:

```text
camera_fx
camera_fy
camera_cx
camera_cy
image_width
image_height
target_fps
```

Nhưng launch file hiện mới truyền:

```text
image_width
image_height
target_fps
```

Nên sửa `real_fractal_detect.launch.py` để expose thêm:

```text
camera_fx
camera_fy
camera_cx
camera_cy
```

Lý do: pose/distance của marker phụ thuộc mạnh vào camera intrinsics. Giá trị fallback hiện tại chỉ phù hợp tương đối cho `1280x720` và HFOV khoảng `81°`:

```text
fx = 749.338
fy = 749.338
cx = 640.0
cy = 360.0
```

Nếu stream SIYI chạy ở `1920x1080`, tối thiểu cần đổi:

```text
image_width  = 1920
image_height = 1080
cx           = 960
cy           = 540
```

`fx`, `fy` nên lấy từ calibration thật.

### 4.3 `rtsp_publisher.py`

File:

```text
~/siyi_ws/src/siyi_camera_bridge/siyi_camera_bridge/rtsp_publisher.py
```

Những điểm cần biết:

- Node đọc RTSP bằng OpenCV/FFmpeg.
- Mặc định ép RTSP transport qua TCP:

```python
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
```

- Publish:

```text
/siyi/image_raw
/siyi/camera_info
```

- CameraInfo hiện giả định distortion bằng 0:

```text
d = [0, 0, 0, 0, 0]
```

Cần sửa nếu:

- RTSP SIYI không dùng path `/main.264`.
- Muốn dùng UDP thay TCP để giảm latency.
- Muốn publish topic khác namespace.
- Muốn dùng camera calibration thật thay vì fallback intrinsics.

Giai đoạn đầu không cần sửa file này, chỉ truyền tham số bằng `--ros-args` hoặc launch.

### 4.4 Tham Số Tracker

Tracker nằm ở:

```text
~/siyi_ws/src/aruco_fractal_tracker
```

Launch hiện truyền các tham số quan trọng:

```text
marker_size
min_tracking_z
max_tracking_z
max_pose_jump_m
acquire_good_frames
lost_bad_frames
camera_x_to_body_east_sign
camera_y_to_body_north_sign
camera_offset_x
camera_offset_y
```

Cần sửa theo UAV thật:

```text
marker_size       Kích thước thật marker ngoài cùng.
max_tracking_z    Khoảng cách detect tối đa mong muốn.
camera_offset_x   Camera lệch trước/sau so với tâm UAV, mét.
camera_offset_y   Camera lệch trái/phải so với tâm UAV, mét.
```

Hiện launch đang giả định SIYI nhìn thẳng xuống:

```text
camera_x_to_body_east_sign = -1.0
camera_y_to_body_north_sign = 1.0
camera_offset_x = 0.0
camera_offset_y = 0.0
```

Nếu gimbal SIYI yaw/pitch động trong lúc detect, mapping này sẽ không đủ. Giai đoạn đầu nên khóa gimbal nhìn thẳng xuống/nadir, rồi detect marker trước.

## 5. Thứ Tự Tích Hợp Khuyến Nghị

### Bước 1: Kiểm Tra RTSP SIYI

Trên UAV:

```bash
ffplay rtsp://<SIYI_IP>:8554/main.264
```

Nếu không có `ffplay`, thử GStreamer:

```bash
gst-launch-1.0 rtspsrc location=rtsp://<SIYI_IP>:8554/main.264 latency=100 ! \
  decodebin ! autovideosink
```

Nếu không mở được, cần kiểm tra:

- IP SIYI.
- UAV có route tới mạng SIYI không.
- RTSP path đúng chưa.
- SIYI camera đã bật stream chưa.

### Bước 2: Chạy Riêng RTSP Bridge

```bash
cd ~/siyi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run siyi_camera_bridge rtsp_publisher --ros-args \
  -p rtsp_url:=rtsp://<SIYI_IP>:8554/main.264 \
  -p flip_180:=true \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p target_fps:=30.0
```

Kiểm tra:

```bash
ros2 topic hz /siyi/image_raw
ros2 topic echo /siyi/camera_info --once
ros2 run rqt_image_view rqt_image_view
```

Trong `rqt_image_view`, chọn:

```text
/siyi/image_raw
```

Nếu ảnh bị ngược, đổi:

```bash
-p flip_180:=false
```

### Bước 3: Chạy Detector Không MAVROS

```bash
cd ~/siyi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=false \
  rtsp_url:=rtsp://<SIYI_IP>:8554/main.264 \
  marker_configuration:=/home/jb/siyi_ws/config/custom_fractal.yml \
  marker_size:=0.50 \
  flip_180:=true
```

Kiểm tra debug image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Chọn:

```text
/siyi/fractal_debug
```

Kiểm tra pose/target:

```bash
ros2 topic echo /siyi/fractal_pose
ros2 topic echo /siyi/landing_target
ros2 topic hz /siyi/landing_target
```

Kỳ vọng khi marker nằm trong ảnh:

```text
/siyi/landing_target state = TRACKING
```

Nếu không detect:

- Kiểm tra đúng marker fractal không.
- Kiểm tra `marker_size`.
- Kiểm tra `custom_fractal.yml` đúng với marker đang dùng.
- Kiểm tra ảnh có bị xoay/ngược không.
- Kiểm tra marker đủ lớn trong ảnh và đủ sáng.

### Bước 4: Kết Nối MAVROS

Nếu UAV thật có Pixhawk/FCU qua USB:

```bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=true \
  fcu_url:=/dev/ttyACM0:57600 \
  rtsp_url:=rtsp://<SIYI_IP>:8554/main.264 \
  marker_configuration:=/home/jb/siyi_ws/config/custom_fractal.yml \
  marker_size:=0.50
```

Nếu đang làm việc với PX4 SITL qua UDP như bài HIL trước:

```bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=true \
  fcu_url:=udp://:14540@<SITL_IP>:14581 \
  rtsp_url:=rtsp://<SIYI_IP>:8554/main.264 \
  marker_configuration:=/home/jb/siyi_ws/config/custom_fractal.yml \
  marker_size:=0.50
```

Nếu MAVROS đã được chạy bởi launch khác, chạy detector với:

```bash
enable_mavros:=false
```

để tránh tạo hai MAVROS node cùng tranh cổng.

## 6. Những Thứ Cần Đo/Hiệu Chỉnh Trên UAV Thật

### Camera Intrinsics

Cần calibrate SIYI ở đúng resolution RTSP dùng cho detect:

```text
image_width
image_height
camera_fx
camera_fy
camera_cx
camera_cy
distortion coefficients
```

Giai đoạn đầu có thể dùng fallback, nhưng distance/pose sẽ chỉ là tương đối.

### Marker Size

Đo cạnh ngoài cùng của marker fractal thật:

```text
marker_size = ... mét
```

Ví dụ marker ngoài cùng 50 cm:

```text
marker_size:=0.50
```

### Camera Offset

Đo camera so với tâm UAV:

```text
camera_offset_x = lệch trước/sau, mét
camera_offset_y = lệch trái/phải, mét
```

Ban đầu có thể để:

```text
camera_offset_x:=0.0
camera_offset_y:=0.0
```

khi test trên bàn hoặc chỉ kiểm tra detect.

### Gimbal Orientation

Pipeline hiện phù hợp nhất khi:

```text
SIYI nhìn thẳng xuống
gimbal yaw không xoay độc lập
pitch/yaw gimbal cố định trong lúc detect
```

Nếu muốn dùng gimbal động, cần thêm tầng transform từ camera frame sang body frame dựa trên attitude thật của gimbal.

## 7. Checklist Trước Khi Bay/Chạy Tích Hợp

```text
[ ] UAV ping được SIYI camera.
[ ] RTSP stream mở được bằng ffplay/gstreamer.
[ ] /siyi/image_raw có frame ổn định.
[ ] /siyi/camera_info đúng width/height.
[ ] /siyi/fractal_debug hiển thị ảnh và overlay.
[ ] /siyi/landing_target chuyển TRACKING khi thấy marker.
[ ] marker_size đúng kích thước thật.
[ ] flip_180 đúng chiều ảnh.
[ ] camera intrinsics tạm chấp nhận hoặc đã calibrate.
[ ] Nếu dùng MAVROS: /mavros/state connected=true.
```

## 8. Các Thay Đổi Code Nên Làm Tiếp

Ưu tiên sửa `real_fractal_detect.launch.py` để truyền được nhiều tham số hơn:

```text
image_width
image_height
target_fps
camera_fx
camera_fy
camera_cx
camera_cy
camera_offset_x
camera_offset_y
```

Sau đó truyền các tham số này vào:

```text
siyi_camera_bridge/rtsp_publisher
aruco_fractal_tracker
```

Lý do: khi lên UAV thật, mỗi lần đổi resolution, lens, gimbal mount, hoặc offset
camera không nên phải sửa code. Chỉ nên đổi launch argument.

## 9. Kết Luận

Để tích hợp dần lên UAV, cần copy tối thiểu ba package:

```text
siyi_camera_bridge
aruco_fractal_tracker
dib_msgs
```

và file:

```text
custom_fractal.yml
```

Những thứ cần thay đổi chính là:

```text
rtsp_url
marker_configuration
marker_size
flip_180
image_width/image_height
camera_fx/fy/cx/cy
camera_offset_x/y
fcu_url hoặc enable_mavros
```

Giai đoạn đầu nên chạy theo thứ tự: RTSP bridge → detector không MAVROS → detector
có MAVROS → tích hợp controller. Cách này tách lỗi rõ ràng giữa camera stream,
detector, và flight/MAVLink.
