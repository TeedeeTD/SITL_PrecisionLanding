# Real Camera Fractal Marker Tracker

Huong dan nay chay pipeline phat hien fractal marker bang camera that SIYI A8 Mini. Project da duoc gom ve mot ROS 2 workspace duy nhat cua repo:

```text
~/PX4/examples/gimbal_simulation/ros2_ws
```

Thu muc `real_camera_tracker/ros2_ws` cu da duoc bo de tranh co hai workspace ROS 2 trong cung project.

## Cau Truc Moi

```text
gimbal_simulation/
├── real_camera_tracker/
│   └── README.md
└── ros2_ws/
    └── src/
        ├── siyi_camera_bridge/       # RTSP -> ROS 2 Image + CameraInfo
        ├── aruco_fractal_tracker/    # Detector fractal ArUco C++
        ├── dib_msgs/                 # LandingTarget6D custom messages
        └── px4_offboard/             # Offboard landing controllers
```

`siyi_camera_bridge` publish:

- `/siyi/image_raw`
- `/siyi/camera_info`

`aruco_fractal_tracker` publish:

- `/siyi/fractal_debug`
- `/siyi/fractal_pose`
- `/siyi/landing_target`

## Build Workspace

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Neu chi muon build lai cac package lien quan camera that:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dib_msgs aruco_fractal_tracker siyi_camera_bridge
source install/setup.bash
```

## Chay Pipeline

### Camera + Detector, khong MAVROS

Dung che do nay khi test tren ban lam viec, chua cam Pixhawk hoac chua chay PX4 SITL/MAVROS:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py enable_mavros:=false
```

### Camera + Detector + MAVROS qua USB

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=true \
  fcu_url:=/dev/ttyACM0:57600
```

### Camera + Detector + MAVROS qua UDP

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=true \
  fcu_url:=udp://:14540@127.0.0.1:14580
```

## Tham So Hay Dung

```bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=false \
  rtsp_url:=rtsp://192.168.168.14:8554/main.264 \
  flip_180:=true \
  marker_size:=0.50
```

Neu PX4 checkout khong nam o `~/PX4`, truyen duong dan marker configuration:

```bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=false \
  marker_configuration:=/absolute/path/to/custom_fractal.yml
```

## Kiem Tra Output

Moi terminal moi can source workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
```

Xem anh camera da xoay 180 do:

```bash
ros2 run rqt_image_view rqt_image_view
```

Chon topic `/siyi/image_raw`.

Xem debug overlay cua tracker:

```bash
ros2 run rqt_image_view rqt_image_view
```

Chon topic `/siyi/fractal_debug`. Overlay se hien `DIST=...m` va `MARKER DIST: ...m`; day la khoang cach truc tiep tu camera den marker, tinh tu vector pose `sqrt(x^2 + y^2 + z^2)`.

Theo doi pose cua marker so voi camera:

```bash
ros2 topic echo /siyi/fractal_pose
```

Khi dua marker ra xa/lai gan, `pose.position.z` va gia tri `DIST` tren `/siyi/fractal_debug` phai thay doi tuong ung theo met.

Theo doi target output cho landing controller:

```bash
ros2 topic echo /siyi/landing_target
```

Topic nay dung `dib_msgs/LandingTarget6D` voi trang thai:

- `LOST = 0`
- `SEARCHING = 1`
- `TRACKING = 2`

Kiem tra tan so camera va tracker:

```bash
ros2 topic hz /siyi/image_raw
ros2 topic hz /siyi/landing_target
```
