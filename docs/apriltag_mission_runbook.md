# AprilTag Precision Landing Mission

This runbook is independent of the ArUco scenario. It only uses:

```text
px4/Tools/simulation/gz/worlds/apriltag_landing.sdf
px4/Tools/simulation/gz/models/apriltag_0
px4/Tools/simulation/gz/models/apriltag_1
px4/Tools/simulation/gz/models/apriltag_2
px4/Tools/simulation/gz/models/apriltag_3
px4/Tools/simulation/gz/models/x500_gimbal
ros2_ws/src/px4_offboard/px4_offboard/apriltag_precision_lander.py
```

It does not require:

```text
px4/Tools/simulation/gz/worlds/aruco_landing.sdf
px4/Tools/simulation/gz/models/arucotag
ros2_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
```

## 1. Prerequisites

First install and verify PX4's native gimbal simulation:

```text
https://docs.px4.io/main/en/advanced/gimbal_control
```

You should already have:

```text
~/PX4
~/px4_ws
ROS 2 Humble
Gazebo Sim
Micro XRCE-DDS Agent
```

Install required ROS/Python packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-bridge \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view \
  python3-opencv

pip3 install pymavlink
```

## 2. Copy PX4 Gazebo Files

From this repository:

```bash
cd ~/gimbal_simulation
```

Copy the AprilTag world:

```bash
cp px4/Tools/simulation/gz/worlds/apriltag_landing.sdf \
  ~/PX4/Tools/simulation/gz/worlds/apriltag_landing.sdf
```

Copy the AprilTag models:

```bash
for id in 0 1 2 3; do
  mkdir -p ~/PX4/Tools/simulation/gz/models/apriltag_${id}
  cp -a px4/Tools/simulation/gz/models/apriltag_${id}/. \
    ~/PX4/Tools/simulation/gz/models/apriltag_${id}/
done
```

Copy the gimbal model used by the mission:

```bash
mkdir -p ~/PX4/Tools/simulation/gz/models/x500_gimbal
cp -a px4/Tools/simulation/gz/models/x500_gimbal/. \
  ~/PX4/Tools/simulation/gz/models/x500_gimbal/
```

Verify:

```bash
ls ~/PX4/Tools/simulation/gz/worlds/apriltag_landing.sdf
ls ~/PX4/Tools/simulation/gz/models/apriltag_0/model.sdf
ls ~/PX4/Tools/simulation/gz/models/apriltag_1/model.sdf
ls ~/PX4/Tools/simulation/gz/models/apriltag_2/model.sdf
ls ~/PX4/Tools/simulation/gz/models/apriltag_3/model.sdf
ls ~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
```

## 3. Copy And Build ROS 2 Package

```bash
mkdir -p ~/px4_ws/src
cp -a ros2_ws/src/px4_offboard ~/px4_ws/src/

cd ~/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard --symlink-install
source ~/px4_ws/install/setup.bash
```

The AprilTag executable is separate from the ArUco executable:

```bash
ros2 pkg executables px4_offboard | grep apriltag_precision_lander
```

Expected:

```text
px4_offboard apriltag_precision_lander
```

## 4. AprilTag Layout

The world contains four selectable `tag25h9` markers:

```text
tag 0: x= 3.0, y= 2.0
tag 1: x= 3.0, y=-2.0
tag 2: x=-3.0, y= 2.0
tag 3: x=-3.0, y=-2.0
```

Select the landing target with:

```bash
--ros-args -p target_tag_id:=0
```

Replace `0` with `1`, `2`, or `3`.

## 5. Run The Mission

Use four terminals.

Terminal 1: PX4 SITL + Gazebo

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

Terminal 3: Gazebo camera bridge

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/world/apriltag_landing/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera" \
  -r "/world/apriltag_landing/clock:=/clock"
```

Terminal 4: AprilTag precision lander

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash
ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=0
```

## 6. Expected State Flow

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

The node ignores non-target tags during correction. Only the selected `target_tag_id` can drive landing correction.

## 7. Camera Viewer

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/gimbal_camera
```

## 8. Stop Everything

```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_bridge|apriltag_precision_lander|rqt_image_view"
```

## 9. FSM Diagram

The FSM is exported here:

```text
docs/apriltag_fsm.svg
docs/apriltag_fsm.png
docs/apriltag_fsm.md
docs/apriltag_fsm.mmd
docs/apriltag_fsm.dot
```
