# X500 Gimbal ArUco Precision Landing Guide

This guide explains how to run the current prepared project from a fresh terminal: PX4 SITL + Gazebo `x500_gimbal`, a ground ArUco landing pad, a ROS 2 camera bridge, and the `px4_offboard` precision landing node.

This is a run and handoff guide, not a chat-history reconstruction. The commands below do not create the custom world, marker model, or ROS 2 landing node. Those files must already exist in the workspace, or they must be copied from this project before running the scenario.

Important: the ArUco landing behavior is not native PX4. The main Python node was created for this project:

```text
~/px4_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
```

PX4 provides the drone, gimbal simulation, Gazebo integration, and reference precision-landing behavior. This project adds the ArUco world, marker fixes, ROS 2 camera bridge workflow, and custom offboard landing controller.

The final behavior is:

```text
takeoff -> pitch gimbal down -> fly to search area -> detect ArUco
-> reposition over marker -> descend while correcting -> land -> disarm
```

## 0. Workspace Assumption

Before running the simulation, confirm that this workspace already contains the project files:

```bash
ls ~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf
ls ~/PX4/Tools/simulation/gz/models/arucotag/model.sdf
ls ~/PX4/Tools/simulation/gz/models/arucotag/arucotag.png
ls ~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
ls ~/px4_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
grep -n "aruco_precision_lander" ~/px4_ws/src/px4_offboard/setup.py
```

If any of those files are missing, the run commands will not reproduce the current result. Copy the missing files from this project first, then rebuild the ROS 2 workspace.

## 1. File Inventory

### Native PX4 Files Used

These already exist in PX4 and are used as the base simulation assets:

```text
~/PX4/Tools/simulation/gz/models/gimbal/model.sdf
~/PX4/Tools/simulation/gz/models/gimbal/model.config
~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
~/PX4/Tools/simulation/gz/models/x500_gimbal/model.config
~/PX4/Tools/simulation/gz/models/arucotag/model.config
```

Main native behavior:

- `gimbal/model.sdf`: defines the gimbal, camera sensor, pitch/yaw/roll joints, and camera frame.
- `x500_gimbal/model.sdf`: integrates the PX4 `x500` drone with the gimbal model.
- `arucotag`: PX4-provided ArUco marker model family.

### Project Files Created Or Modified For This Work

```text
~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf
~/PX4/Tools/simulation/gz/models/arucotag/model.sdf
~/PX4/Tools/simulation/gz/models/arucotag/arucotag.png
~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
~/px4_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
~/px4_ws/src/px4_offboard/setup.py
~/px4_ws/src/px4_offboard/package.xml
~/PX4/aruco_precision_landing_gimbal.md
```

Implementation status:

| File | Status | Purpose |
| --- | --- | --- |
| `Tools/simulation/gz/worlds/aruco_landing.sdf` | Created | New Gazebo world for this scenario. It places the ArUco landing pad at local position `x=3.0`, `y=2.0`, `z=0.0`. |
| `Tools/simulation/gz/models/arucotag/model.sdf` | Modified | Keeps marker physical size at `0.5m x 0.5m`, adds a thin raised base/collision, and lifts the visual plane to avoid blurry z-fighting with the ground. |
| `Tools/simulation/gz/models/arucotag/arucotag.png` | Modified/regenerated | Sharper ArUco texture used by Gazebo so OpenCV can detect the marker reliably. |
| `Tools/simulation/gz/models/x500_gimbal/model.sdf` | Modified | Adjusts the included gimbal pose on the `x500` body. |
| `px4_offboard/px4_offboard/aruco_precision_lander.py` | Created | Main custom ROS 2 node. Handles takeoff, gimbal pitch, ArUco detection, target correction, descent, landing, and disarm. |
| `px4_offboard/setup.py` | Modified | Registers the node as the executable `ros2 run px4_offboard aruco_precision_lander`. |
| `px4_offboard/package.xml` | Modified/required | Declares runtime dependencies such as `rclpy`, `px4_msgs`, `sensor_msgs`, `cv_bridge`, and OpenCV. |
| `aruco_precision_landing_gimbal.md` | Created | This guide. |

Key implementation details:

- `aruco_landing.sdf`: new Gazebo world with the ArUco pad at local position `x=3.0`, `y=2.0`.
- `arucotag/model.sdf`: marker kept at original physical size `0.5m x 0.5m`, with a thin raised base to avoid ground z-fighting.
- `arucotag/arucotag.png`: regenerated as a sharper marker texture so the camera can detect it reliably.
- `x500_gimbal/model.sdf`: gimbal include pose adjusted to the current working pose.
- `aruco_precision_lander.py`: new ROS 2 offboard controller for ArUco detection, gimbal pitch, repositioning, descent, landing, and disarm. This file did not come from PX4.
- `setup.py`: must contain this console script entry:

```python
'aruco_precision_lander = px4_offboard.aruco_precision_lander:main',
```

- `package.xml`: must include dependencies needed by the node:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>px4_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
<exec_depend>opencv2</exec_depend>
```

- `aruco_precision_landing_gimbal.md`: this guide.

These files are the implementation deliverables. The terminal commands later in this guide only run them.

### If Recreating This In A Clean Workspace

Starting from a clean PX4 checkout is not enough. You must bring over the project implementation files first:

```text
copy/create: ~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf
copy/update: ~/PX4/Tools/simulation/gz/models/arucotag/model.sdf
copy/update: ~/PX4/Tools/simulation/gz/models/arucotag/arucotag.png
copy/update: ~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
copy/create: ~/px4_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
update:      ~/px4_ws/src/px4_offboard/setup.py
check:       ~/px4_ws/src/px4_offboard/package.xml
```

Then rebuild:

```bash
cd ~/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard --symlink-install
```

Current gimbal include pose:

```xml
<pose>0.085 0 0.28 0 0 3.14</pose>
```

File:

```text
~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
```

Meaning:

```text
x: forward/back
y: left/right
z: up/down
roll pitch yaw: current yaw is 3.14 rad
```

## 2. Prerequisites

Use Ubuntu with ROS 2 Humble, PX4 SITL, Gazebo, QGroundControl, and the `~/px4_ws` ROS 2 workspace.

Required packages/tools:

```text
PX4-Autopilot workspace: ~/PX4
ROS 2 workspace: ~/px4_ws
ros_gz_bridge
Micro XRCE-DDS Agent
QGroundControl
OpenCV with cv2.aruco support
pymavlink
```

If `pymavlink` is missing:

```bash
pip3 install pymavlink
```

## 3. Build The ROS 2 Package

Run this after editing the landing node:

```bash
cd ~/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard --symlink-install
```

Then source the workspace:

```bash
source ~/px4_ws/install/setup.bash
```

If you see RTPS/DDS payload-size errors, your `px4_msgs` definitions do not match the PX4 firmware. Re-sync PX4 messages into `~/px4_ws`, then rebuild:

```bash
cd ~/px4_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 4. Terminal Layout

Use four required terminals:

```text
Terminal 1: PX4 SITL + Gazebo
Terminal 2: Micro XRCE-DDS Agent
Terminal 3: Gazebo camera/clock bridge
Terminal 4: ArUco precision landing node
```

Optional:

```text
Terminal 5: rqt_image_view camera viewer
```

## 5. Step 1: Start PX4 + Gazebo ArUco World

Terminal 1:

```bash
cd ~/PX4
PX4_GZ_WORLD=aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

Expected PX4/Gazebo setup:

```text
world: aruco_landing
vehicle model: x500_gimbal_0
landing pad model: landing_pad
ArUco model source: model://arucotag
ArUco pad position: x=3.0, y=2.0, z=0.0
```

`PX4_GZ_NO_FOLLOW=1` keeps the Gazebo camera interactive, so you can manually inspect the drone and marker.

If you only want to test the native PX4 gimbal simulation without the ArUco world:

```bash
cd ~/PX4
PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

That launches the default world, not the ArUco landing world.

## 6. Step 2: Start Micro XRCE-DDS Agent

Terminal 2:

```bash
MicroXRCEAgent udp4 -p 8888
```

If your executable is lowercase:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

This connects PX4 uORB topics to ROS 2 DDS topics. The precision lander needs it for:

```text
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v4
/fmu/out/vehicle_attitude
/fmu/out/vehicle_land_detected
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

## 7. Step 3: Bridge Gazebo Camera And Clock

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

Important:

- Use `/world/aruco_landing/...` for this project.
- Use `/world/default/...` only when you launch the default PX4 world.
- The node subscribes to `/gimbal_camera`.
- The node uses simulation time through `/clock`.

Optional bridge config equivalent:

```yaml
- ros_topic_name: "/gimbal_camera"
  gz_topic_name: "/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/clock"
  gz_topic_name: "/world/aruco_landing/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

## 8. Step 4a: View Camera Stream In ROS

Terminal 5:

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/gimbal_camera
```

If there is no image, check Terminal 3 bridge topics and confirm the world name is `aruco_landing`.

## 9. Step 4b: Optional View In QGroundControl

QGroundControl video is not required for autonomous landing, but it is useful for debugging the gimbal camera.

Install GStreamer tools:

```bash
sudo apt install ros-humble-gscam gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav
```

For the native gimbal camera workflow, stream a UDP H264 feed to QGroundControl port `5600`. Configure QGroundControl video source as UDP H264 on port `5600`.

Note: the precision landing node does not depend on this QGroundControl video path. It reads directly from `/gimbal_camera`.

## 10. Step 5: Run The ArUco Precision Lander

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash
ros2 run px4_offboard aruco_precision_lander
```

Expected log sequence:

```text
MAVLink connected
ArucoPrecisionLander ready
State: INIT -> TAKEOFF
Switching to Offboard mode...
Arming...
Takeoff complete at ~5m - pitching gimbal down
State: TAKEOFF -> GIMBAL_DOWN
Gimbal settle complete - flying to search area
State: GIMBAL_DOWN -> FLY_TO_SEARCH
Arrived at search area - starting ArUco search
ArUco DETECTED
State: SEARCH -> HORIZONTAL_APPROACH
Horizontal approach: visual_error=...
PX4 horizontal acceptance reached - descending over target
State: HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET
Descend over target: alt=..., visual_error=...
State: DESCEND_OVER_TARGET -> FINAL_APPROACH
State: FINAL_APPROACH -> LAND
LANDING COMPLETE
```

## 11. Step 6: Stop All Processes

Use this when restarting the full test:

```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_bridge|aruco_precision_lander|rqt_image_view"
```

## 12. Manual Gimbal Control

PX4 gimbal commands from the PX4 shell or QGroundControl MAVLink Console:

```bash
gimbal status
gimbal test pitch -30
gimbal test pitch -45
gimbal test yaw 60
gimbal stop
```

Approximate limits:

```text
pitch: -2.4 to 0.8 rad
yaw:   -3.14 to 3.14 rad
```

Gazebo joint topic commands, mainly for debugging:

```bash
gz topic -t "/model/x500_gimbal_0/command/gimbal_pitch" \
  -m gz.msgs.Double -p "data: -0.5"

gz topic -t "/model/x500_gimbal_0/command/gimbal_yaw" \
  -m gz.msgs.Double -p "data: 0.5"
```

In this project, `aruco_precision_lander.py` commands the gimbal through PX4 vehicle commands:

```text
VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE
VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW
VEHICLE_CMD_DO_MOUNT_CONTROL
```

This is more reliable here than direct `gz topic` control, which timed out during earlier testing.

## 13. Landing Controller Logic

The ROS 2 node is:

```text
~/px4_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
```

Mission states:

```text
INIT
-> TAKEOFF
-> GIMBAL_DOWN
-> FLY_TO_SEARCH
-> SEARCH
-> HORIZONTAL_APPROACH
-> DESCEND_OVER_TARGET
-> TARGET_LOST, only if needed
-> FINAL_APPROACH
-> LAND
-> DONE
```

Important constants:

```python
CRUISE_ALT = 5.0
SEARCH_POS = (3.0, 2.0)
MARKER_SIZE = 0.50

PLD_HACC_RAD = 0.25
PLD_BTOUT = 2.0
PLD_FAPPR_ALT = 0.10
PLD_MAX_SRCH = 3
PLD_SRCH_ALT = 5.0

DESCENT_LOSS_GRACE = 8.0
FORCE_DISARM_DELAY = 8.0
```

How repositioning works:

1. Detect ArUco marker corners in `/gimbal_camera`.
2. Compute marker center in image pixels.
3. Compare marker center to image center.
4. Convert pixel error into north/east ground offset using current altitude and camera field of view.
5. Move the UAV horizontally until marker visual error is below `PLD_HACC_RAD`.
6. Start descent only after horizontal acceptance.
7. During descent, keep correcting while the marker remains visible.
8. If marker detection drops during descent, continue on the last good target for `DESCENT_LOSS_GRACE`.
9. Below `PLD_FAPPR_ALT`, finish landing even if the marker is lost.
10. At ground height, command land and disarm.

Disarm behavior:

- First tries normal PX4 disarm.
- If PX4 says `not landed` while Gazebo shows the vehicle on the pad, waits `FORCE_DISARM_DELAY`.
- Then sends PX4 force-disarm using `param2=21196`.

## 14. Why `target lost` Can Appear

This log is acceptable after descent has already started:

```text
target lost during descent - continuing on last target
Target temporarily lost: continuing descent on last target
Target reacquired - continuing descent
```

Common reasons:

- the marker becomes too close to the camera;
- part of the marker leaves the camera frame;
- landing gear or vehicle body occludes the marker;
- camera blur briefly breaks ArUco detection;
- the marker is near the edge of the image.

Safety rule:

- If target is lost during initial `HORIZONTAL_APPROACH`, the node does not blindly descend.
- If target is lost during `DESCEND_OVER_TARGET`, the node can continue briefly because it was already centered first.
- If target is lost during `FINAL_APPROACH`, the node finishes the landing because it is already near the ground.

## 15. Troubleshooting

### No Camera Image

Check the bridge world name:

```text
/world/aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image
```

If you launched the default world, then the topic is:

```text
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image
```

### ArUco Detected But UAV Moves Away

Watch:

```text
Horizontal approach: visual_error=...
target=[x, y]
```

If `visual_error` grows instead of shrinking, the image-to-NED sign mapping is wrong for the current camera pose. Fix `_marker_image_rel_ned()` in `aruco_precision_lander.py`.

### `Disarming denied: not landed`

Gazebo can show touchdown while PX4 land detector still reports `not landed`, especially on the raised ArUco pad. The node handles this with delayed force-disarm at ground height.

Expected fallback log:

```text
PX4 still reports not landed - force disarming at ground height
LANDING COMPLETE
```

### DDS Payload Size Error

If you see:

```text
RTPS_READER_HISTORY Error: Change payload size ...
```

then ROS 2 `px4_msgs` does not match the PX4 firmware message definitions. Re-sync messages and rebuild `~/px4_ws`.

### OpenCV ArUco Pose Helper Missing

If you see:

```text
module 'cv2.aruco' has no attribute 'estimatePoseSingleMarkers'
```

this node does not need that helper. It estimates pose with `cv2.solvePnP()`.

## 16. Results

Current verified result:

```text
Simulation starts successfully.
Gimbal pitches down automatically after takeoff.
Camera detects the ArUco marker.
UAV repositions over the marker.
UAV descends while correcting lateral error.
Temporary target loss during descent is tolerated.
UAV lands on the ArUco pad.
Node disarms after landing, with force-disarm fallback for SITL land-detector mismatch.
```

## 17. References

PX4 documentation:

- Gimbal control: https://docs.px4.io/main/en/advanced/gimbal_control
- Precision landing: https://docs.px4.io/main/en/advanced_features/precland
- Vision Target Estimator: https://docs.px4.io/main/en/advanced_features/vision_target_estimator

Local implementation files:

```text
~/PX4/Tools/simulation/gz/models/gimbal/model.sdf
~/PX4/Tools/simulation/gz/models/x500_gimbal/model.sdf
~/PX4/Tools/simulation/gz/models/arucotag/model.sdf
~/PX4/Tools/simulation/gz/models/arucotag/arucotag.png
~/PX4/Tools/simulation/gz/worlds/aruco_landing.sdf
~/px4_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
```
