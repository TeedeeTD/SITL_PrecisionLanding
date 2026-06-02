# Gimbal Precision Landing Simulation

https://github.com/do010303/gimbal_simulation

This project adds precision landing examples for PX4 Gazebo `x500_gimbal`.

PX4 provides the base drone, gimbal model, Gazebo simulation, and gimbal control stack. This repository adds custom landing-pad worlds, marker models, and ROS 2 `px4_offboard` landing nodes.

The landing nodes are custom code for this project. They are not native PX4.

## 1. Install Base PX4 Gimbal Simulation

First follow the PX4 gimbal simulation guide:

```text
https://docs.px4.io/main/en/advanced/gimbal_control
```

At the end you should be able to run:

```bash
cd ~/PX4
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

You should also have ROS 2 Humble and a ROS workspace:

```text
~/PX4
~/px4_ws
```

## 2. Install Required ROS/Python Packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-bridge \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view \
  python3-opencv
```

Install MAVLink Python support:

```bash
pip3 install pymavlink
```

Install or verify Micro XRCE-DDS Agent. Depending on your PX4 setup, one of these commands should exist:

```bash
MicroXRCEAgent udp4 -p 8888
```

or:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

## 3. Clone This Repository

```bash
cd ~
git clone git@github.com:do010303/gimbal_simulation.git
cd ~/gimbal_simulation
```

If SSH is not configured:

```bash
git clone https://github.com/do010303/gimbal_simulation.git
cd ~/gimbal_simulation
```

## 4. Copy PX4 Simulation Files

Copy the custom Gazebo worlds:

```bash
cp px4/Tools/simulation/gz/worlds/aruco_landing.sdf \
  ~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf

cp px4/Tools/simulation/gz/worlds/apriltag_landing.sdf \
  ~/PX4/Tools/simulation/gz/worlds/apriltag_landing.sdf
```

Copy the ArUco marker model:

```bash
mkdir -p ~/PX4/Tools/simulation/gz/models/arucotag
cp -a px4/Tools/simulation/gz/models/arucotag/. \
  ~/PX4/Tools/simulation/gz/models/arucotag/
```

Copy the four AprilTag marker models:

```bash
for id in 0 1 2 3; do
  mkdir -p ~/PX4/Tools/simulation/gz/models/apriltag_${id}
  cp -a px4/Tools/simulation/gz/models/apriltag_${id}/. \
    ~/PX4/Tools/simulation/gz/models/apriltag_${id}/
done
```

Copy the modified `x500_gimbal` model:

```bash
mkdir -p ~/PX4/Tools/simulation/gz/models/x500_gimbal
cp -a px4/Tools/simulation/gz/models/x500_gimbal/. \
  ~/PX4/Tools/simulation/gz/models/x500_gimbal/
```

Verify:

```bash
ls ~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf
ls ~/PX4/Tools/simulation/gz/worlds/apriltag_landing.sdf
ls ~/PX4/Tools/simulation/gz/models/arucotag/model.sdf
ls ~/PX4/Tools/simulation/gz/models/apriltag_0/model.sdf
ls ~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
```

## 5. Copy ROS 2 Package

Copy the provided `px4_offboard` package into your ROS workspace:

```bash
mkdir -p ~/px4_ws/src
cp -a ros2_ws/src/px4_offboard ~/px4_ws/src/
```

The main custom nodes are:

```text
~/px4_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
~/px4_ws/src/px4_offboard/px4_offboard/apriltag_precision_lander.py
```

The package entry points are registered in:

```text
~/px4_ws/src/px4_offboard/setup.py
```

as:

```python
aruco_precision_lander = px4_offboard.aruco_precision_lander:main
apriltag_precision_lander = px4_offboard.apriltag_precision_lander:main
```

## 6. Build ROS 2 Workspace

```bash
cd ~/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard --symlink-install
source ~/px4_ws/install/setup.bash
```

If you see DDS payload size errors later, your `px4_msgs` package does not match your PX4 checkout. Re-sync PX4 messages and rebuild the workspace.

## 7. Run AprilTag Landing

The AprilTag world has four `tag25h9` markers. Select the landing target with `target_tag_id`.

```text
tag 0: x= 3.0, y= 2.0
tag 1: x= 3.0, y=-2.0
tag 2: x=-3.0, y= 2.0
tag 3: x=-3.0, y=-2.0
```

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

Terminal 3: Gazebo camera and clock bridge

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/world/apriltag_landing/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera" \
  -r "/world/apriltag_landing/clock:=/clock"
```

Terminal 4: AprilTag precision landing node

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash
ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=0
```

Change `target_tag_id` to `1`, `2`, or `3` to land on another tag. After takeoff, the node searches visually from the UAV's current position instead of flying directly to a fixed target coordinate.

The AprilTag node uses visible tags for relative navigation. For example, if the target is tag `1` but the camera sees tag `0` or `2`, the node combines that tag's camera pose with the known tag-to-tag spacing to estimate the relative direction to tag `1`. If no useful tag is visible, it runs an expanding square search around the current or last estimated target position.

If the node prints only the ready lines and never prints `State: INIT -> TAKEOFF`, the control loop is not ticking. Rebuild the workspace after copying the latest node:

```bash
cd ~/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard --symlink-install
source ~/px4_ws/install/setup.bash
```

Also confirm Terminal 3 is running, because `/gimbal_camera` is required for detection:

```bash
ros2 topic hz /gimbal_camera
```

## 8. Run ArUco Landing

The original ArUco scenario is still included.

Terminal 1:

```bash
cd ~/PX4
PX4_GZ_WORLD=aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Terminal 3 bridge:

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/world/aruco_landing/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera" \
  -r "/world/aruco_landing/clock:=/clock"
```

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash
ros2 run px4_offboard aruco_precision_lander
```

## 9. Optional Camera Viewer

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/gimbal_camera
```

## 10. Expected Flow

```text
INIT
-> TAKEOFF
-> GIMBAL_DOWN
-> FLY_TO_SEARCH
-> SEARCH
-> HORIZONTAL_APPROACH
-> DESCEND_OVER_TARGET
-> FINAL_APPROACH
-> LAND
-> DONE
```

Temporary `target lost during descent` logs are acceptable after the UAV has already reached `DESCEND_OVER_TARGET`. The node continues on the last good target for a short grace period and reacquires if the marker returns.

## 11. Stop Everything

```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_bridge|aruco_precision_lander|apriltag_precision_lander|rqt_image_view"
```

## 12. Notes

- AprilTag uses OpenCV's `DICT_APRILTAG_25h9` dictionary.
- Each AprilTag pad is `0.5m x 0.5m`.
- AprilTag generally gives stronger detection under blur, perspective distortion, and longer range than ArUco, which is why it is a better fit for this landing test.
