# Gimbal ArUco Precision Landing

This project adds an ArUco precision landing scenario for PX4 Gazebo `x500_gimbal`.

PX4 provides the base drone, gimbal model, Gazebo simulation, and gimbal control stack. This repository adds:

- an `aruco_landing` Gazebo world;
- a corrected ArUco landing pad model/texture;
- a ROS 2 `px4_offboard` node named `aruco_precision_lander`;
- a runbook in `docs/full_runbook.md`.

The landing node is custom code for this project. It is not native PX4.

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

Install bridge and image packages:

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

Copy the custom Gazebo world:

```bash
cp px4/Tools/simulation/gz/worlds/aruco_landing.sdf \
  ~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf
```

Copy the modified ArUco marker model:

```bash
mkdir -p ~/PX4/Tools/simulation/gz/models/arucotag
cp -a px4/Tools/simulation/gz/models/arucotag/. \
  ~/PX4/Tools/simulation/gz/models/arucotag/
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
ls ~/PX4/Tools/simulation/gz/models/arucotag/model.sdf
ls ~/PX4/Tools/simulation/gz/models/arucotag/arucotag.png
ls ~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
```

## 5. Copy ROS 2 Package

Copy the provided `px4_offboard` package into your ROS workspace:

```bash
mkdir -p ~/px4_ws/src
cp -a ros2_ws/src/px4_offboard ~/px4_ws/src/
```

The main custom node is:

```text
~/px4_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
```

The package entry point is registered in:

```text
~/px4_ws/src/px4_offboard/setup.py
```

as:

```python
aruco_precision_lander = px4_offboard.aruco_precision_lander:main
```

## 6. Build ROS 2 Workspace

```bash
cd ~/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard --symlink-install
source ~/px4_ws/install/setup.bash
```

If you see DDS payload size errors later, your `px4_msgs` package does not match your PX4 checkout. Re-sync PX4 messages and rebuild the workspace.

## 7. Run The Simulation

Use four terminals.

Terminal 1: PX4 SITL + Gazebo

```bash
cd ~/PX4
PX4_GZ_WORLD=aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
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
  "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/world/aruco_landing/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera" \
  -r "/world/aruco_landing/clock:=/clock"
```

Terminal 4: precision landing node

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash
ros2 run px4_offboard aruco_precision_lander
```

## 8. Optional Camera Viewer

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/gimbal_camera
```

## 9. Expected Flow

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

## 10. Stop Everything

```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_bridge|aruco_precision_lander|rqt_image_view"
```

## 11. Notes

- The ArUco pad is spawned at local position `x=3.0`, `y=2.0`.
- The marker physical size is `0.5m x 0.5m`.
- The gimbal is commanded down automatically after takeoff.
- The final disarm includes a SITL force-disarm fallback because PX4 can visually touch down in Gazebo while still reporting `not landed`.
- For more implementation detail, read `docs/full_runbook.md`.
