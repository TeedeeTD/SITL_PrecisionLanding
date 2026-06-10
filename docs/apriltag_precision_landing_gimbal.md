# X500 Gimbal AprilTag Precision Landing Report

This document is the combined runbook and technical report for the current
AprilTag precision landing mission. It explains what was built, which native
PX4/Gazebo files were reused, which files were created for this project, how
the control code works, and how to reproduce the run from a clean machine.

The current mission is AprilTag-first and independent from the older ArUco
mission. The ArUco files are still kept as a legacy/optional example, but they
are not required to run the AprilTag landing node.

## 1. Objective

The objective is to make a simulated PX4 `x500_gimbal` UAV:

```text
take off -> point camera downward -> search for a selected AprilTag
-> reposition above that selected tag -> descend while correcting XY drift
-> finish landing -> disarm
```

The final mission supports four selectable AprilTags:

```text
tag 0: x= 3.0, y= 2.0
tag 1: x= 3.0, y=-2.0
tag 2: x=-3.0, y= 2.0
tag 3: x=-3.0, y=-2.0
```

The landing target is selected at runtime:

```bash
ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=0
```

Replace `0` with `1`, `2`, or `3` to land on another tag.

## 2. Final Result

The final AprilTag mission can land on the selected tag and ignore other visible
tags. This matters because, with four markers in the world, detecting "any tag"
is not enough. If the controller allows non-target tags to update the landing
target, the UAV can chase the wrong marker or oscillate between estimates.

The final controller therefore uses this rule:

```text
Detect all visible AprilTags, but only target_tag_id is allowed to drive
horizontal correction and descent.
```

The final behavior is:

- The UAV takes off to `5.0 m`.
- The gimbal is commanded to look downward.
- The UAV performs a camera-based expanding search pattern from its current
  position.
- When the selected tag is detected, the node estimates the selected tag's
  image-center offset.
- The UAV moves horizontally until the selected tag is centered.
- The UAV descends while still correcting XY drift.
- Near `0.1 m`, the node switches to normal PX4 landing/disarm handling.

## 3. Project Scope

This repository is a subset project for the precision landing mission:

```text
examples/gimbal_simulation
```

Despite the folder name, the current main mission is AprilTag. The name was kept
because the project started as an ArUco/gimbal landing experiment.

The subset project is meant to be copied into a normal PX4 and ROS 2 setup:

```text
~/PX4
~/PX4/examples/gimbal_simulation/ros2_ws
```

It does not replace a full PX4 checkout. It provides the custom world, marker
models, ROS 2 node, and documentation needed to reproduce the mission.

## 4. Native PX4/Gazebo Support Used

PX4 already provides most of the simulation and flight infrastructure. This
project uses the following native PX4 features instead of rebuilding them:

| Native component | Used for |
| --- | --- |
| `make px4_sitl gz_x500_gimbal` | Starts PX4 SITL with the native Gazebo x500 gimbal model. |
| `Tools/simulation/gz/models/gimbal` | Provides the simulated gimbal, camera sensor, joints, and camera topic. |
| `Tools/simulation/gz/models/x500_gimbal` | Provides the x500 airframe with the gimbal attached. |
| PX4 Offboard mode | Lets the ROS 2 node publish position setpoints. |
| PX4 `VehicleCommand` | Used for arm, disarm, mode switch, gimbal commands, and normal land. |
| PX4 local position topics | Used by the node to know current NED position and altitude. |
| PX4 vehicle attitude topic | Used to rotate visual correction into the local NED frame. |
| PX4 land detector topic | Used to decide when a normal disarm is accepted. |
| PX4 gimbal manager/mount commands | Used to request `-90 deg` gimbal pitch after takeoff. |
| MAVLink `LANDING_TARGET` | Used to report a vision fiducial target in a PX4-compatible format. |
| Micro XRCE-DDS Agent | Bridges PX4 uORB data to ROS 2 topics. |
| `ros_gz_bridge` | Bridges Gazebo camera frames into ROS 2. |

The relevant PX4 references are:

```text
https://docs.px4.io/main/en/advanced/gimbal_control
https://docs.px4.io/main/en/advanced_features/precland
https://docs.px4.io/main/en/advanced_features/vision_target_estimator
https://docs.px4.io/main/en/advanced_config/land_detector
```

## 5. Project Files Created Or Modified

The AprilTag mission files are separate from the ArUco mission files.

### 5.1 AprilTag Mission Files

These files are required for the AprilTag mission:

```text
px4/Tools/simulation/gz/worlds/apriltag_landing.sdf
px4/Tools/simulation/gz/models/apriltag_0/model.sdf
px4/Tools/simulation/gz/models/apriltag_0/model.config
px4/Tools/simulation/gz/models/apriltag_0/tag25h9-0.png
px4/Tools/simulation/gz/models/apriltag_1/model.sdf
px4/Tools/simulation/gz/models/apriltag_1/model.config
px4/Tools/simulation/gz/models/apriltag_1/tag25h9-1.png
px4/Tools/simulation/gz/models/apriltag_2/model.sdf
px4/Tools/simulation/gz/models/apriltag_2/model.config
px4/Tools/simulation/gz/models/apriltag_2/tag25h9-2.png
px4/Tools/simulation/gz/models/apriltag_3/model.sdf
px4/Tools/simulation/gz/models/apriltag_3/model.config
px4/Tools/simulation/gz/models/apriltag_3/tag25h9-3.png
px4/Tools/simulation/gz/models/x500_gimbal/model.sdf
px4/Tools/simulation/gz/models/x500_gimbal/model.config
ros2_ws/src/px4_offboard/px4_offboard/apriltag_precision_lander.py
ros2_ws/src/px4_offboard/setup.py
ros2_ws/src/px4_offboard/package.xml
```

Purpose of each main file:

| File | Status | Purpose |
| --- | --- | --- |
| `apriltag_landing.sdf` | Created | Gazebo world with four selectable AprilTag landing pads. |
| `apriltag_0..3/model.sdf` | Created | Marker models with raised tag visuals and collision geometry. |
| `tag25h9-0..3.png` | Created from AprilTag references | Textures for AprilTag family `tag25h9`. |
| `x500_gimbal/model.sdf` | Copied/kept in subset | Gimbal UAV model used by the mission. |
| `apriltag_precision_lander.py` | Created | Main ROS 2 precision landing controller. |
| `setup.py` | Modified | Registers `apriltag_precision_lander` as a ROS 2 executable. |
| `package.xml` | Modified/checked | Declares runtime dependencies used by the node. |

### 5.2 Optional ArUco Files

These are kept only for the earlier ArUco experiment:

```text
px4/Tools/simulation/gz/worlds/aruco_landing.sdf
px4/Tools/simulation/gz/models/arucotag/model.sdf
px4/Tools/simulation/gz/models/arucotag/model.config
px4/Tools/simulation/gz/models/arucotag/arucotag.png
ros2_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
```

The AprilTag mission does not depend on those files.

## 6. Why Move From ArUco To AprilTag

The project began with one ArUco marker. That version proved the basic sequence:

```text
takeoff -> detect marker -> reposition -> descend -> land
```

However, the simulation exposed several practical issues:

- The marker texture could appear blurry when spawned flat on the ground.
- Ground z-fighting made the marker hard to detect.
- The camera/gimbal orientation had to be controlled consistently.
- With a single marker, the mission did not test target selection.

AprilTag was selected for the next version because it is generally stronger for
fiducial detection under perspective changes, distance, partial blur, and
multi-marker scenes. The current project uses OpenCV's AprilTag dictionary:

```python
aruco.DICT_APRILTAG_25h9
```

This gave a clean path to four selectable targets while still using OpenCV's
`cv2.aruco` API.

## 7. How The AprilTag World Works

The AprilTag world is:

```text
px4/Tools/simulation/gz/worlds/apriltag_landing.sdf
```

It creates:

- a grey ground plane,
- simple lighting,
- PX4-compatible spherical coordinates,
- four included marker models.

Marker placement:

```xml
<include>
  <uri>model://apriltag_0</uri>
  <name>landing_pad_tag_0</name>
  <pose>3.0 2.0 0 0 0 0</pose>
</include>
```

The other tags are placed at `(3, -2)`, `(-3, 2)`, and `(-3, -2)`.

The marker models use PNG textures:

```text
tag25h9-0.png
tag25h9-1.png
tag25h9-2.png
tag25h9-3.png
```

Each marker is modeled as a physical pad, not only a texture. This prevents the
visual from being completely merged into the ground plane and makes the marker
more stable in Gazebo rendering.

## 8. ROS 2 Package Structure

The ROS 2 package is:

```text
ros2_ws/src/px4_offboard
```

Important files:

```text
package.xml
setup.py
px4_offboard/apriltag_precision_lander.py
px4_offboard/aruco_precision_lander.py
px4_offboard/camera_viewer.py
px4_offboard/drone_controller.py
```

The AprilTag and ArUco nodes are separate executables in the same package:

```python
'aruco_precision_lander = px4_offboard.aruco_precision_lander:main',
'apriltag_precision_lander = px4_offboard.apriltag_precision_lander:main',
```

This means the AprilTag mission can be run separately:

```bash
ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=0
```

and the old ArUco mission can still be run separately:

```bash
ros2 run px4_offboard aruco_precision_lander
```

## 9. Communication Architecture

The runtime communication chain is:

```text
Gazebo camera
  -> ros_gz_bridge
  -> /gimbal_camera
  -> apriltag_precision_lander.py

PX4 SITL uORB
  -> Micro XRCE-DDS Agent
  -> px4_msgs ROS 2 topics
  -> apriltag_precision_lander.py

apriltag_precision_lander.py
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
  -> /fmu/in/vehicle_command
  -> MAVLink LANDING_TARGET udpout:127.0.0.1:14540
```

The node subscribes to:

```text
/gimbal_camera
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v4
/fmu/out/vehicle_attitude
/fmu/out/vehicle_land_detected
```

The node publishes to:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

The node also sends MAVLink:

```text
LANDING_TARGET
```

through:

```text
udpout:127.0.0.1:14540
```

## 10. AprilTag Detection Pipeline

The detector is built from OpenCV:

```python
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_25h9)
detector = aruco.ArucoDetector(aruco_dict, params)
```

Camera frames arrive on:

```text
/gimbal_camera
```

The image callback does:

```text
ROS Image -> cv_bridge -> BGR image -> grayscale
-> detect AprilTag corners and IDs
-> estimate pose with solvePnP
-> store all visible tags
-> select only target_tag_id for control
```

The code detects all tags because that helps diagnostics, but it only updates
the active landing estimate when the selected tag is visible:

```text
visible tags: [0, 1, 2]
target_tag_id: 1
only tag 1 updates marker_center_px, marker_tvec, and target_abs
```

If the selected tag is not visible, non-target tags are logged but ignored. This
was one of the key fixes that made multi-tag landing stable.

The node uses `cv2.solvePnP` instead of `estimatePoseSingleMarkers`. This was
done because some OpenCV builds do not provide `estimatePoseSingleMarkers`.
Using `solvePnP` keeps the node compatible with the available ROS/OpenCV setup.

The control correction is based primarily on marker image-center error, not raw
`tvec`. The `tvec` is still useful for logging and debugging.

## 11. Position Correction Method

The downward camera sees the marker as a 2D image target. The node compares the
selected tag center to the image center:

```text
dx_px = marker_center_x - camera_center_x
dy_px = marker_center_y - camera_center_y
```

Then it estimates how many meters that pixel error represents at the current
altitude:

```text
meters_per_px_x = camera_ground_width_at_altitude / image_width
meters_per_px_y = camera_ground_height_at_altitude / image_height
```

For a downward-looking camera:

```text
image x error -> east/right correction
image y error -> north/south correction
```

The vehicle yaw is used to rotate that correction into local NED:

```text
camera/vehicle-relative correction -> local NED correction
```

The target estimate is then smoothed:

```text
target_abs = old_target * (1 - alpha) + measured_target * alpha
```

where:

```text
VISION_TARGET_ALPHA = 0.25
```

This prevents the UAV from chasing noisy single-frame detections.

## 12. Search Strategy

Earlier versions used a fixed search coordinate. That worked only when the UAV
started near the expected marker location. It was not flexible enough if the UAV
started farther away.

The current AprilTag mission searches visually from the UAV's current position.
It uses an expanding pattern around a search anchor:

```text
anchor -> +x -> +x,+y -> +y -> -x,+y -> -x -> -x,-y -> -y -> +x,-y
```

The search radius increases by ring:

```text
SEARCH_PATTERN_STEP = 1.0 m
SEARCH_PATTERN_MAX_RADIUS = 5.0 m
```

This means the UAV does not assume the selected tag is already under the fixed
world coordinate. It scans the area until the camera actually sees the selected
tag.

The predefined tag positions still exist in the world and in the config, but
the final controller does not blindly land on those coordinates. It uses them as
known references/fallbacks, while the visual detector is the main source for
landing correction.

## 13. Finite State Machine

The AprilTag node uses this state machine:

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

It also has a recovery state:

```text
TARGET_LOST
```


### INIT

Purpose:

```text
Publish Offboard setpoints for a short warmup period before arming.
```

Why:

PX4 Offboard mode requires a valid setpoint stream before switching into
Offboard. Without this warmup, PX4 may reject Offboard or immediately failsafe.

### TAKEOFF

Purpose:

```text
Switch to Offboard, arm, and climb to CRUISE_ALT.
```

Main values:

```text
CRUISE_ALT = 5.0 m
```

When the altitude error is small enough, the state changes to `GIMBAL_DOWN`.

### GIMBAL_DOWN

Purpose:

```text
Command the gimbal pitch to -90 deg so the camera looks at the ground.
```

The node uses PX4 gimbal/mount command paths:

```text
VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE
VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW
VEHICLE_CMD_DO_MOUNT_CONTROL
```

The gimbal command is repeated while waiting for settle time. This was added
because a single gimbal command could be missed or overridden during startup.

### SEARCH

Purpose:

```text
Run the camera-based search pattern until the selected target_tag_id is visible.
```

The node checks:

```text
camera frames are arriving
AprilTag IDs are detected
selected target_tag_id is visible
```

If other tags are visible but the target is not, the node logs the visible IDs
and keeps searching.

### HORIZONTAL_APPROACH

Purpose:

```text
Move horizontally until the selected tag is centered under the UAV.
```

The node:

- updates the target from the selected tag's image-center error,
- sends `LANDING_TARGET`,
- moves the setpoint toward the visual target,
- holds altitude at `CRUISE_ALT`,
- requires the target to stay centered for several cycles.

Main acceptance value:

```text
PLD_HACC_RAD = 0.25 m
CENTER_CONFIRM_COUNT = 8 cycles
```

This is inspired by PX4 precision landing horizontal acceptance behavior.

### DESCEND_OVER_TARGET

Purpose:

```text
Descend while still correcting XY drift.
```

The node continues to update the target from the selected tag. If the visual
error grows too much, the node returns to `HORIZONTAL_APPROACH` instead of
continuing a bad descent.

Main value:

```text
DESCENT_HACC_RAD = 0.35 m
DESCENT_RATE = 0.45 m/s
```

### TARGET_LOST

Purpose:

```text
Handle temporary loss of the selected tag.
```

This is expected in the simulation, especially at lower altitudes when the tag
can temporarily leave the camera frame or be partially blocked by the UAV view.

During horizontal approach, loss means:

```text
pause/research until the selected tag is reacquired
```

During descent, loss means:

```text
continue briefly on the last good target
```

Main values:

```text
PLD_BTOUT = 2.0 s
DESCENT_LOSS_GRACE = 8.0 s
LOST_TARGET_DESCENT_RATE = 0.25 m/s
```

This was added after testing showed that immediately aborting descent on every
small detection flicker made the UAV climb/down-loop instead of landing.

### FINAL_APPROACH

Purpose:

```text
Finish descent near the ground even if the tag disappears.
```

At very low altitude, the camera may not reliably see the whole marker. The
node therefore accepts the last good target and descends to the final altitude.

Main value:

```text
FINAL_ALT = 0.10 m
PLD_FAPPR_ALT = 0.10 m
```

### LAND

Purpose:

```text
Let PX4 finish normal landing and disarm.
```

The node sends:

```text
VEHICLE_CMD_NAV_LAND
```

Then it waits for either:

```text
PX4 land detector says landed
```

Only after PX4 reports `landed=true` does the node send a normal disarm command.
The node no longer treats low altitude alone as proof of touchdown, because that
can disarm a real vehicle while it is still flying.

Force disarm is disabled by default. It exists only as an explicit simulation or
bench-test escape hatch:

```text
allow_force_disarm = false
```

Do not enable `allow_force_disarm` on real hardware.

### DONE

Purpose:

```text
Mission is complete. No more active control commands are needed.
```

## 14. Relationship To PX4 Native Precision Landing

PX4 has native precision landing support through the precision landing and
vision target estimator features. The project uses those documents as the
design reference, especially for:

```text
horizontal acceptance radius
target timeout behavior
final approach altitude
search/reacquire behavior
land detector handling
LANDING_TARGET reporting
```

However, the final implementation does not fully hand control to native PX4
precision landing at the beginning of descent. That approach was tested and was
less stable in this setup because:

- the mission has multiple visible tags,
- the selected tag must be enforced by ID,
- temporary loss of the target is common near the ground,
- the custom node already has direct camera-frame access and can keep correcting
  with the selected tag only.

The final approach is therefore hybrid:

```text
PX4 provides simulation, Offboard mode, land detector, gimbal command path,
and normal landing/disarm behavior.

The ROS 2 node performs selected-tag visual search, centering, descent logic,
and recovery behavior.

The node streams MAVLink LANDING_TARGET so the target estimate is reported in a
PX4-compatible way.
```

## 15. Installation Guide

### 15.1 Prerequisites

First install and verify PX4's native gimbal simulation:

```text
https://docs.px4.io/main/en/advanced/gimbal_control
```

You should already have:

```text
~/PX4
~/PX4/examples/gimbal_simulation
ROS 2 Humble
Gazebo Sim
Micro XRCE-DDS Agent
```

Install ROS/Python packages:

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

### 15.2 Clone The Subset Repository Into PX4

```bash
cd ~/PX4
mkdir -p examples
git clone git@github.com:do010303/gimbal_simulation.git examples/gimbal_simulation
```

If SSH is not configured:

```bash
cd ~/PX4
mkdir -p examples
git clone https://github.com/do010303/gimbal_simulation.git examples/gimbal_simulation
```

### 15.3 Sync PX4 Simulation Files

Sync the Gazebo world and model overlay into PX4:

```bash
cd ~/PX4
rsync -a \
  examples/gimbal_simulation/px4/Tools/simulation/gz/ \
  Tools/simulation/gz/
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

### 15.4 Build ROS 2 Package In Place

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Verify the AprilTag executable:

```bash
ros2 pkg executables px4_offboard | grep apriltag_precision_lander
```

Expected:

```text
px4_offboard apriltag_precision_lander
```

## 16. Running The AprilTag Mission

Use four terminals.

### Terminal 1: PX4 SITL + Gazebo

```bash
cd ~/PX4
PX4_GZ_WORLD=apriltag_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

### Terminal 2: Micro XRCE-DDS Agent

Use whichever command exists on your system:

```bash
MicroXRCEAgent udp4 -p 8888
```

or:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

### Terminal 3: Gazebo Camera Bridge

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/world/apriltag_landing/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/gimbal_camera" \
  -r "/world/apriltag_landing/clock:=/clock"
```

### Terminal 4: AprilTag Precision Lander

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=0
```

To land on another marker:

```bash
ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=1
ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=2
ros2 run px4_offboard apriltag_precision_lander --ros-args -p target_tag_id:=3
```

## 17. Viewing The Camera

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/gimbal_camera
```

Check camera frequency:

```bash
ros2 topic hz /gimbal_camera
```

## 18. Stopping The Simulation

```bash
pkill -9 -f "gz sim|px4|MicroXRCEAgent|micro-xrce-dds-agent|ros_gz_bridge|apriltag_precision_lander|rqt_image_view"
```

## 19. Expected Console Output

A good run should show this approximate sequence:

```text
AprilTagPrecisionLander ready - state: INIT
State: INIT -> TAKEOFF
Switching to Offboard mode...
Arming...
Takeoff complete at 4.7m - pitching gimbal down
State: TAKEOFF -> GIMBAL_DOWN
Gimbal settle complete - starting camera-based tag search
State: GIMBAL_DOWN -> SEARCH
Selected AprilTag 0 DETECTED
State: SEARCH -> HORIZONTAL_APPROACH
Horizontal approach: visual_error=...
PX4 horizontal acceptance reached - descending over target
State: HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET
Descend over target: alt=...
PX4 final-approach altitude reached
State: DESCEND_OVER_TARGET -> FINAL_APPROACH
0.1m reached - switching to normal land/disarm
State: FINAL_APPROACH -> LAND
LANDING COMPLETE
State: LAND -> DONE
```

Temporary target loss during descent is not automatically a failure:

```text
target lost during descent - continuing on last target for 8.0s
Target reacquired - continuing descent
```

That means the tag flickered out of the camera view and the node continued with
the last good target estimate.

## 21. Troubleshooting

### Node starts but never takes off

Check that the ROS 2 workspace was rebuilt after copying the package:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Check that PX4 and Micro XRCE-DDS Agent are running.

### No camera frames

Symptom:

```text
No /gimbal_camera frames received.
```

Check the bridge command uses:

```text
/world/apriltag_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image
```

Check the topic:

```bash
ros2 topic list | grep gimbal_camera
ros2 topic hz /gimbal_camera
```

### Detects a tag but not the selected tag

The node may log:

```text
Target tag 1 not in view; visible=[0, 2]
```

This means the camera sees AprilTags, but not the one selected by
`target_tag_id`. Either choose a visible tag ID or let the search pattern move
until the selected tag appears.

### MAVLink `vehicle_command_ack lost`

This can appear in PX4 SITL when command acknowledgements are overwritten or
dropped during high command activity. If the mission continues through the FSM,
it is usually not the root problem. If the node does not arm or switch modes,
restart PX4, the DDS agent, and the ROS node.

### DDS payload size error

If ROS prints payload-size errors, the local `px4_msgs` package likely does not
match the PX4 checkout. Re-sync/rebuild `px4_msgs` and rebuild
`~/PX4/examples/gimbal_simulation/ros2_ws`.

### QGroundControl says "not landed"

The simulation can visually touch the ground before PX4's land detector accepts
the landed state. The node now waits for `vehicle_land_detected.landed` before
sending normal disarm. If this waits forever, inspect PX4 land-detector signals
instead of force-disarming: `ground_contact`, `maybe_landed`, `at_rest`,
vertical movement, horizontal movement, and local-position altitude.


## 22. References

PX4 references used:

```text
PX4 Gimbal Control
https://docs.px4.io/main/en/advanced/gimbal_control

PX4 Precision Landing
https://docs.px4.io/main/en/advanced_features/precland

PX4 Vision Target Estimator
https://docs.px4.io/main/en/advanced_features/vision_target_estimator

PX4 Land Detector
https://docs.px4.io/main/en/advanced_config/land_detector
```

Project repository:

```text
https://github.com/do010303/gimbal_simulation
```

## 23. Appendix: AprilTag-Only File Checklist

Required for AprilTag:

```text
px4/Tools/simulation/gz/worlds/apriltag_landing.sdf
px4/Tools/simulation/gz/models/apriltag_0
px4/Tools/simulation/gz/models/apriltag_1
px4/Tools/simulation/gz/models/apriltag_2
px4/Tools/simulation/gz/models/apriltag_3
px4/Tools/simulation/gz/models/x500_gimbal
ros2_ws/src/px4_offboard/px4_offboard/apriltag_precision_lander.py
ros2_ws/src/px4_offboard/setup.py
ros2_ws/src/px4_offboard/package.xml
```

Not required for AprilTag:

```text
px4/Tools/simulation/gz/worlds/aruco_landing.sdf
px4/Tools/simulation/gz/models/arucotag
ros2_ws/src/px4_offboard/px4_offboard/aruco_precision_lander.py
```
