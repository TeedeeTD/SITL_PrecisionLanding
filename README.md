# ArUco Precision Landing Subset Project

This folder is a self-contained copy of the custom files needed for the X500 gimbal ArUco precision landing scenario.

It does not replace the normal PX4 workflow. On this laptop, the scenario can still be run with the existing files in `~/PX4` and `~/px4_ws`. This folder exists so someone else can pull the git repo and install the same custom files into their own PX4 and ROS 2 workspaces.

## Prerequisite

First complete the normal PX4 gimbal simulation setup:

```text
https://docs.px4.io/main/en/advanced/gimbal_control
```

Expected local workspaces:

```text
~/PX4
~/px4_ws
```

Expected ROS/PX4 tools:

```text
ROS 2 Humble
Gazebo / gz sim
ros_gz_bridge
Micro XRCE-DDS Agent
QGroundControl
cv_bridge
OpenCV aruco
pymavlink
```

## What This Subset Contains

PX4 simulation files:

```text
px4/Tools/simulation/gz/worlds/aruco_landing.sdf
px4/Tools/simulation/gz/models/arucotag/model.sdf
px4/Tools/simulation/gz/models/arucotag/model.config
px4/Tools/simulation/gz/models/arucotag/arucotag.png
px4/Tools/simulation/gz/models/x500_gimbal/model.sdf
px4/Tools/simulation/gz/models/x500_gimbal/model.config
```

ROS 2 package files:

```text
ros2_ws/src/px4_offboard/package.xml
ros2_ws/src/px4_offboard/setup.py
ros2_ws/src/px4_offboard/setup.cfg
ros2_ws/src/px4_offboard/resource/px4_offboard
ros2_ws/src/px4_offboard/px4_offboard/__init__.py
ros2_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
ros2_ws/src/px4_offboard/px4_offboard/camera_viewer.py
ros2_ws/src/px4_offboard/px4_offboard/drone_controller.py
```

The main custom file is:

```text
ros2_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
```

That file was created for this project. It is not native PX4.

## Install Into Local Workspaces

From this folder:

```bash
./install_into_workspace.sh
```

The install script copies files into `~/PX4` and `~/px4_ws`. It will overwrite the matching project files listed above.

By default it installs into:

```text
PX4_DIR=~/PX4
ROS_WS=~/px4_ws
```

Override paths if needed:

```bash
PX4_DIR=/path/to/PX4 ROS_WS=/path/to/px4_ws ./install_into_workspace.sh
```

Then rebuild the ROS 2 package:

```bash
cd ~/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard --symlink-install
source ~/px4_ws/install/setup.bash
```

## Run

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

## Expected Flow

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

## Stop Everything

```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_bridge|aruco_precision_lander|rqt_image_view"
```

## Notes

- The ArUco pad is spawned at local position `x=3.0`, `y=2.0`.
- The marker physical size is `0.5m x 0.5m`.
- The gimbal is commanded down automatically after takeoff.
- The final disarm includes a SITL force-disarm fallback because PX4 can visually touch down in Gazebo while still reporting `not landed`.
