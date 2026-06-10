# PX4 Gimbal Precision Landing

This example adds visual precision landing missions for the PX4 Gazebo
`x500_gimbal` vehicle.

PX4 provides the simulator, vehicle, gimbal stack, Gazebo bridge, and base
gimbal control behavior. This example adds:

- AprilTag and ArUco landing worlds
- marker models used by those worlds
- a ROS 2 `px4_offboard` package with precision landing nodes

The landing nodes are project code, not built-in PX4 modules.

## Recommended Layout

Use this example from inside a full PX4 checkout:

```text
~/PX4
└── examples
    └── gimbal_simulation
```

That layout lets you build the ROS 2 workspace in place. However, PX4 Gazebo
still loads worlds and models from:

```text
~/PX4/Tools/simulation/gz/worlds
~/PX4/Tools/simulation/gz/models
```

So cloning this example into `~/PX4/examples` is not enough by itself. You must
sync the provided Gazebo overlay into `~/PX4/Tools/simulation/gz` once, or after
changing the models/worlds.

## Prerequisites

First install and verify normal PX4 Gazebo simulation:

```text
https://docs.px4.io/main/en/simulation/
```

Then follow the PX4 gimbal simulation guide:

```text
https://docs.px4.io/main/en/advanced/gimbal_control
```

Before continuing, this should work:

```bash
cd ~/PX4
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

You also need ROS 2 Humble, `colcon`, the Micro XRCE-DDS Agent, and the ROS/Gazebo bridge packages.

### Installing ROS 2 Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-bridge \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view \
  python3-colcon-common-extensions \
  python3-opencv

pip3 install pymavlink
```

### Installing Micro XRCE-DDS Agent
To enable communication between PX4 and ROS 2, you must install the Micro XRCE-DDS Agent.

#### Option A: Build from Source (Recommended)
Building from source is recommended as it avoids sandbox/network restrictions and works reliably with localhost-only configurations:
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

#### Option B: Install via Snap (Alternative)
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

### Important: Localhost Restriction Settings (ROS_LOCALHOST_ONLY)
* If you run with `ROS_LOCALHOST_ONLY=0` (or leave it unset), DDS discovery will work out of the box.
* **If you enforce `export ROS_LOCALHOST_ONLY=1`**, you **must** configure the DDS participant in PX4 to also restrict itself to localhost, otherwise the ROS 2 nodes will not receive any topics.
  To do this, start PX4 and in the PX4 console (`pxh>`) run:
  ```bash
  param set UXRCE_DDS_PTCFG 1
  ```
  Then restart PX4.

## Install This Example

From a full PX4 checkout, clone or place this repository at the expected path:

```bash
cd ~/PX4
mkdir -p examples
git clone https://github.com/do010303/gimbal_simulation.git \
  examples/gimbal_simulation
```

If you already have the folder, just make sure the path is exactly:

```text
~/PX4/examples/gimbal_simulation
```

Sync the Gazebo world and model overlay into PX4:

```bash
cd ~/PX4
rsync -a \
  examples/gimbal_simulation/px4/Tools/simulation/gz/ \
  Tools/simulation/gz/
```

Verify the important files are now visible to PX4:

```bash
ls Tools/simulation/gz/worlds/apriltag_landing.sdf
ls Tools/simulation/gz/worlds/aruco_landing.sdf
ls Tools/simulation/gz/models/apriltag_0/model.sdf
ls Tools/simulation/gz/models/x500_gimbal/model.sdf
```

## Build the ROS 2 Workspace

The ROS 2 workspace lives inside this example:

```text
~/PX4/examples/gimbal_simulation/ros2_ws
```

The package depends on `px4_msgs`. If your PX4 ROS 2 setup already provides
`px4_msgs`, source that workspace before building this one. Otherwise, clone
`px4_msgs` into this workspace:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
```

Build:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you later change any Python node, rebuild or keep using
`--symlink-install`.

## Run AprilTag Precision Landing

The AprilTag world has four `tag25h9` landing targets:

```text
tag 0: x= 3.0, y= 2.0
tag 1: x= 3.0, y=-2.0
tag 2: x=-3.0, y= 2.0
tag 3: x=-3.0, y=-2.0
```

Open four terminals.

Terminal 1: PX4 SITL and Gazebo

```bash
cd ~/PX4
PX4_GZ_WORLD=apriltag_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Terminal 2: Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

or:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

Terminal 3: camera and clock bridge

```bash
source /opt/ros/humble/setup.bash

gz topic -l | grep -E "camera.*/image|/image$"

ros2 run ros_gz_image image_bridge \
  "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image" \
  --ros-args \
  -r "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera"
```

The `gz topic -l` line should print the same camera image topic used by the
`image_bridge` command. If your Gazebo version prints a different camera topic,
replace the hard-coded image topic in both places in the `image_bridge` command
before starting Terminal 4.

Terminal 4: landing node

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash

ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=0
```

Change `target_tag_id` to `1`, `2`, or `3` to land on another pad.

## Run ArUco Precision Landing

The original single-marker ArUco scenario is also included.

Terminal 1:

```bash
cd ~/PX4
PX4_GZ_WORLD=aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Terminal 2:

```bash
MicroXRCEAgent udp4 -p 8888
```

or:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash

gz topic -l | grep -E "camera.*/image|/image$"

ros2 run ros_gz_image image_bridge \
  "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image" \
  --ros-args \
  -r "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera"
```

The `gz topic -l` line should print the same camera image topic used by the
`image_bridge` command. If your Gazebo version prints a different camera topic,
replace the hard-coded image topic in both places in the `image_bridge` command
before starting Terminal 4.

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash

ros2 run px4_offboard aruco_precision_lander
```

## Optional Camera Viewer

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/gimbal_camera
```

## Expected State Flow

```text
INIT
-> TAKEOFF
-> GIMBAL_DOWN
-> SEARCH
-> HORIZONTAL_APPROACH
-> DESCEND_OVER_TARGET
-> FINAL_APPROACH
-> LAND
-> DONE
```

Temporary target-loss logs are normal if the marker leaves the camera view for a
few frames.

## Troubleshooting

### Stuck in `INIT`
If the landing node starts but never leaves `INIT`, check that the camera topic is active and publishing:
```bash
ros2 topic hz /gimbal_camera
```

### Stuck in `TAKEOFF` (printing `Arming...` or `Switching to Offboard node...` continuously)
This happens when the ROS 2 node does not receive vehicle state updates from PX4, or PX4 rejects the arming/offboard commands.

1. **Check ROS 2 connectivity**:
   Verify if the status topic is actively publishing:
   ```bash
   ros2 topic echo /fmu/out/vehicle_status_v4
   ```
   If it displays nothing, the DDS discovery between ROS 2 and the Micro XRCE-DDS Agent is failing.

2. **DDS Localhost Mismatch (`ROS_LOCALHOST_ONLY`)**:
   If you have `ROS_LOCALHOST_ONLY=1` exported in your terminals, the ROS 2 nodes will only listen on the loopback (`127.0.0.1`) interface. You **must** also configure the PX4 client to only use loopback:
   * Run the simulation.
   * In the PX4 console (`pxh>`), set the parameter:
     ```bash
     param set UXRCE_DDS_PTCFG 1
     ```
   * Restart the simulation.

3. **Check Client Status in PX4**:
   In the PX4 console (`pxh>`), check the client connection status:
   ```bash
   uxrce_dds_client status
   ```

### World File Not Found
If PX4 says the world file does not exist, run the Gazebo overlay sync again:
```bash
cd ~/PX4
rsync -a \
  examples/gimbal_simulation/px4/Tools/simulation/gz/ \
  Tools/simulation/gz/
```

### Leftover Processes
If you see errors like `PX4 server already running for instance 0` or port conflicts, kill all leftover PX4, Gazebo, and ROS 2 processes:
```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_bridge|aruco_precision_lander|apriltag_precision_lander|rqt_image_view"
```

## Notes

- AprilTag uses OpenCV's `DICT_APRILTAG_25h9` dictionary.
- Each AprilTag pad is `0.5 m x 0.5 m`.
- The AprilTag node ignores non-target tags during correction so the target
  estimate does not jump between pads.
- The Python node performs visual centering in Offboard mode and also streams
  MAVLink `LANDING_TARGET` for PX4 precision-landing compatibility.

More detailed design notes are in:

```text
docs/apriltag_precision_landing_gimbal.md
docs/apriltag_fsm.md
docs/camera_cm4_rtsp_runbook.md
```

## Appendix: Install PX4 and Verify Gimbal Simulation

Use this appendix only if you do not already have a working PX4 checkout with
Gazebo simulation.

Official references:

```text
https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu
https://docs.px4.io/main/en/sim_gazebo_gz/
https://docs.px4.io/main/en/advanced/gimbal_control
```

PX4 currently targets Ubuntu 24.04 LTS for CI and release builds, with Ubuntu
22.04 LTS also supported. PX4's Ubuntu setup script installs the normal
simulation tools; Gazebo Harmonic is installed by default on Ubuntu 22.04, and
Gazebo Harmonic/Ionic/Jetty are supported on Ubuntu 24.04.

Clone PX4:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive PX4
```

Install the PX4 toolchain:

```bash
bash ~/PX4/Tools/setup/ubuntu.sh
```

Restart the computer after the script finishes. Then build and run a normal
Gazebo SITL vehicle:

```bash
cd ~/PX4
make px4_sitl gz_x500
```

Stop it with `Ctrl+C`, then verify the gimbal vehicle:

```bash
cd ~/PX4
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

If this command opens Gazebo and spawns `x500_gimbal_0`, the PX4 side is ready
for this example. Then continue from `Install This Example` above.

If you see:

```text
ninja: error: unknown target 'gz_x500'
```

clean and retry:

```bash
cd ~/PX4
make distclean
make px4_sitl gz_x500
```

# Hạ cánh chính xác với PX4 Gimbal

Ví dụ này thêm các bài bay hạ cánh chính xác bằng thị giác cho PX4 Gazebo
vehicle `x500_gimbal`.

PX4 cung cấp simulator, vehicle, gimbal stack, Gazebo bridge, và hành vi điều
khiển gimbal cơ bản. Ví dụ này thêm:

- world hạ cánh AprilTag và ArUco
- marker model dùng trong các world đó
- ROS 2 package `px4_offboard` với các node hạ cánh chính xác

Các landing node là code riêng của project, không phải module có sẵn trong PX4.

## Cấu trúc thư mục khuyến nghị

Dùng ví dụ này bên trong một PX4 checkout đầy đủ:

```text
~/PX4
└── examples
    └── gimbal_simulation
```

Cấu trúc này cho phép build ROS 2 workspace ngay tại chỗ. Tuy nhiên, PX4 Gazebo
vẫn load world và model từ:

```text
~/PX4/Tools/simulation/gz/worlds
~/PX4/Tools/simulation/gz/models
```

Vì vậy, chỉ clone ví dụ này vào `~/PX4/examples` là chưa đủ. Bạn cần sync Gazebo
overlay được cung cấp vào `~/PX4/Tools/simulation/gz` một lần, hoặc sau khi thay
đổi model/world.

## Yêu cầu trước khi chạy

Trước tiên hãy cài đặt và kiểm tra PX4 Gazebo simulation bình thường:

```text
https://docs.px4.io/main/en/simulation/
```

Sau đó làm theo hướng dẫn PX4 gimbal simulation:

```text
https://docs.px4.io/main/en/advanced/gimbal_control
```

Trước khi tiếp tục, lệnh này cần chạy được:

```bash
cd ~/PX4
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Bạn cũng cần ROS 2 Humble, `colcon`, Micro XRCE-DDS Agent, và các package ROS/Gazebo bridge.

### Cài đặt các package ROS 2
```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-bridge \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view \
  python3-colcon-common-extensions \
  python3-opencv

pip3 install pymavlink
```

### Cài đặt Micro XRCE-DDS Agent
Để kết nối truyền thông giữa PX4 và ROS 2, bạn cần cài đặt Micro XRCE-DDS Agent.

#### Cách A: Cài đặt từ mã nguồn (Khuyên dùng)
Cài từ source giúp tránh các lỗi sandbox/bảo mật mạng của Snap và hỗ trợ tốt cấu hình chỉ dùng localhost:
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

#### Cách B: Cài đặt qua Snap (Thay thế)
```bash
sudo snap install micro-xrce-dds-agent --classic
```

Sau khi cài đặt, bạn có thể chạy Agent bằng lệnh:
```bash
MicroXRCEAgent udp4 -p 8888
```
hoặc:
```bash
micro-xrce-dds-agent udp4 -p 8888
```

### Lưu ý quan trọng về chế độ Localhost (`ROS_LOCALHOST_ONLY`)
* Nếu bạn để mặc định `ROS_LOCALHOST_ONLY=0` (hoặc không set), hệ thống sẽ kết nối bình thường ngay lập tức.
* **Nếu bạn set `export ROS_LOCALHOST_ONLY=1`**, bạn **bắt buộc** phải cấu hình PX4 chỉ sử dụng localhost để đồng bộ.
  Hãy bật PX4 lên, gõ lệnh sau trong terminal `pxh>`:
  ```bash
  param set UXRCE_DDS_PTCFG 1
  ```
  Sau đó khởi động lại PX4.

## Cài ví dụ này

Từ một PX4 checkout đầy đủ, clone hoặc đặt repository này vào đúng đường dẫn:

```bash
cd ~/PX4
mkdir -p examples
git clone https://github.com/do010303/gimbal_simulation.git \
  examples/gimbal_simulation
```

Nếu bạn đã có sẵn folder, hãy đảm bảo đường dẫn chính xác là:

```text
~/PX4/examples/gimbal_simulation
```

Sync Gazebo world và model overlay vào PX4:

```bash
cd ~/PX4
rsync -a \
  examples/gimbal_simulation/px4/Tools/simulation/gz/ \
  Tools/simulation/gz/
```

Kiểm tra các file quan trọng đã nằm ở nơi PX4 có thể thấy:

```bash
ls Tools/simulation/gz/worlds/apriltag_landing.sdf
ls Tools/simulation/gz/worlds/aruco_landing.sdf
ls Tools/simulation/gz/models/apriltag_0/model.sdf
ls Tools/simulation/gz/models/x500_gimbal/model.sdf
```

## Build ROS 2 Workspace

ROS 2 workspace nằm bên trong ví dụ này:

```text
~/PX4/examples/gimbal_simulation/ros2_ws
```

Package này phụ thuộc vào `px4_msgs`. Nếu setup ROS 2 cho PX4 của bạn đã cung
cấp `px4_msgs`, hãy source workspace đó trước khi build workspace này. Nếu chưa,
clone `px4_msgs` vào workspace này:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
```

Build:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Nếu sau này bạn sửa Python node, hãy build lại hoặc tiếp tục dùng
`--symlink-install`.

## Chạy AprilTag Precision Landing

World AprilTag có bốn target hạ cánh `tag25h9`:

```text
tag 0: x= 3.0, y= 2.0
tag 1: x= 3.0, y=-2.0
tag 2: x=-3.0, y= 2.0
tag 3: x=-3.0, y=-2.0
```

Mở bốn terminal.

Terminal 1: PX4 SITL và Gazebo

```bash
cd ~/PX4
PX4_GZ_WORLD=apriltag_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Terminal 2: Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

hoặc:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

Terminal 3: bridge camera và clock

```bash
source /opt/ros/humble/setup.bash

gz topic -l | grep -E "camera.*/image|/image$"

ros2 run ros_gz_image image_bridge \
  "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image" \
  --ros-args \
  -r "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera"
```

Dòng `gz topic -l` phải in ra đúng camera image topic đang dùng trong lệnh
`image_bridge`. Nếu Gazebo version của bạn in ra camera topic khác, hãy thay
image topic hard-code ở cả hai vị trí trong lệnh `image_bridge` trước khi chạy
Terminal 4.

Terminal 4: landing node

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash

ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=0
```

Đổi `target_tag_id` thành `1`, `2`, hoặc `3` để hạ cánh lên pad khác.

## Chạy ArUco Precision Landing

Kịch bản ArUco một marker ban đầu vẫn được giữ lại.

Terminal 1:

```bash
cd ~/PX4
PX4_GZ_WORLD=aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Terminal 2:

```bash
MicroXRCEAgent udp4 -p 8888
```

hoặc:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash

gz topic -l | grep -E "camera.*/image|/image$"

ros2 run ros_gz_image image_bridge \
  "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image" \
  --ros-args \
  -r "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera"
```

Dòng `gz topic -l` phải in ra đúng camera image topic đang dùng trong lệnh
`image_bridge`. Nếu Gazebo version của bạn in ra camera topic khác, hãy thay
image topic hard-code ở cả hai vị trí trong lệnh `image_bridge` trước khi chạy
Terminal 4.

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash

ros2 run px4_offboard aruco_precision_lander
```

## Xem camera tùy chọn

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Chọn:

```text
/gimbal_camera
```

## Luồng trạng thái dự kiến

```text
INIT
-> TAKEOFF
-> GIMBAL_DOWN
-> SEARCH
-> HORIZONTAL_APPROACH
-> DESCEND_OVER_TARGET
-> FINAL_APPROACH
-> LAND
-> DONE
```

Log mất target tạm thời là bình thường nếu marker rời khỏi camera view trong
vài frame.

## Xử lý lỗi

### Bị kẹt ở `INIT`
Nếu landing node chạy nhưng không rời khỏi trạng thái `INIT`, hãy kiểm tra xem camera topic đã hoạt động và đang publish hay chưa:
```bash
ros2 topic hz /gimbal_camera
```

### Bị kẹt ở `TAKEOFF` (liên tục in `Arming...` hoặc `Switching to Offboard node...`)
Lỗi này xảy ra khi node ROS 2 không nhận được thông tin trạng thái từ PX4, hoặc PX4 từ chối lệnh Arm/Offboard.

1. **Kiểm tra kết nối ROS 2**:
   Xem thử có nhận được dữ liệu trạng thái từ PX4 hay không:
   ```bash
   ros2 topic echo /fmu/out/vehicle_status_v4
   ```
   Nếu lệnh trên không hiển thị gì, nghĩa là quá trình DDS discovery giữa ROS 2 và Micro XRCE-DDS Agent đang thất bại.

2. **Đồng bộ chế độ Localhost (`ROS_LOCALHOST_ONLY`)**:
   Nếu bạn có `export ROS_LOCALHOST_ONLY=1` ở các terminal, các node ROS 2 chỉ giao tiếp trên card loopback `127.0.0.1`. Bạn **bắt buộc** phải cấu hình PX4 dùng card loopback tương ứng:
   * Chạy mô phỏng lên.
   * Gõ lệnh sau trong terminal PX4 (`pxh>`):
     ```bash
     param set UXRCE_DDS_PTCFG 1
     ```
   * Tắt đi và chạy lại mô phỏng.

3. **Kiểm tra trạng thái Client trên PX4**:
   Gõ lệnh sau trong terminal PX4 (`pxh>`) để kiểm tra tình trạng kết nối:
   ```bash
   uxrce_dds_client status
   ```

### Không tìm thấy World file
Nếu PX4 báo không tìm thấy file world, hãy thực hiện sync lại Gazebo overlay:
```bash
cd ~/PX4
rsync -a \
  examples/gimbal_simulation/px4/Tools/simulation/gz/ \
  Tools/simulation/gz/
```

### Dọn dẹp tiến trình chạy ngầm
Nếu bạn gặp lỗi `PX4 server already running for instance 0` hoặc xung đột port, hãy tắt hết các tiến trình cũ còn sót:
```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_bridge|aruco_precision_lander|apriltag_precision_lander|rqt_image_view"
```

### Lỗi DDS payload / message type
Nếu ROS 2 báo lỗi DDS payload hoặc message type, package `px4_msgs` của bạn có
thể không khớp với PX4 checkout. Hãy re-sync hoặc rebuild workspace cung cấp
`px4_msgs`, rồi rebuild workspace này.

### Ghi chú về cơ chế disarm
Landing node mặc định không force-disarm. Trong state `LAND`, node gửi PX4
`NAV_LAND`, dừng stream Offboard setpoint, chờ
`vehicle_land_detected.landed`, rồi mới gửi lệnh disarm bình thường. Nếu node
liên tục in `Waiting for PX4 land detector before disarm`, hãy sửa vấn đề land
detector hoặc altitude/contact trước khi chạy trên phần cứng thật.

## Ghi chú

- AprilTag dùng OpenCV dictionary `DICT_APRILTAG_25h9`.
- Mỗi AprilTag pad có kích thước `0.5 m x 0.5 m`.
- AprilTag node bỏ qua các tag không phải target trong lúc correction để target
  estimate không nhảy giữa các pad.
- Python node thực hiện visual centering trong Offboard mode và cũng stream
  MAVLink `LANDING_TARGET` để tương thích với PX4 precision landing.

Ghi chú thiết kế chi tiết hơn nằm ở:

```text
docs/apriltag_precision_landing_gimbal.md
docs/apriltag_fsm.md
docs/camera_cm4_rtsp_runbook.md
```

## Phụ lục: Cài đặt PX4 và kiểm tra mô phỏng gimbal

Chỉ dùng phần này nếu bạn chưa có PX4 checkout chạy được Gazebo simulation.

Tài liệu chính thức:

```text
https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu
https://docs.px4.io/main/en/sim_gazebo_gz/
https://docs.px4.io/main/en/advanced/gimbal_control
```

PX4 hiện dùng Ubuntu 24.04 LTS cho CI/release build và vẫn hỗ trợ Ubuntu 22.04
LTS. Script cài đặt của PX4 sẽ cài các công cụ mô phỏng cần thiết. Trên Ubuntu
22.04, Gazebo Harmonic được cài mặc định; trên Ubuntu 24.04, PX4 hỗ trợ Gazebo
Harmonic/Ionic/Jetty.

Clone PX4:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive PX4
```

Cài toolchain của PX4:

```bash
bash ~/PX4/Tools/setup/ubuntu.sh
```

Sau khi script chạy xong, hãy restart máy. Sau đó build và chạy thử Gazebo SITL
cơ bản:

```bash
cd ~/PX4
make px4_sitl gz_x500
```

Dừng bằng `Ctrl+C`, rồi kiểm tra vehicle có gimbal:

```bash
cd ~/PX4
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Nếu Gazebo mở lên và spawn được `x500_gimbal_0`, phần PX4 đã sẵn sàng cho
example này. Sau đó quay lại mục `Cài ví dụ này` ở phía trên.

Nếu gặp lỗi:

```text
ninja: error: unknown target 'gz_x500'
```

hãy clean rồi chạy lại:

```bash
cd ~/PX4
make distclean
make px4_sitl gz_x500
```
