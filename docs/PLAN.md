# Execution Plan: Requirement-by-Requirement Precision Landing

## Summary

- Push target is the repo at `~/PX4/examples/gimbal_simulation`, not the parent PX4-Autopilot repo.
- README update was pushed on request. Future git pushes must happen only when explicitly requested.
- The current flight flow is kept: `Offboard search/center/descend -> NAV_LAND -> PX4 land detector -> normal disarm`.
- Components are implemented one at a time with a test gate after each step.
- Current simulation code uses `pymavlink`, not MAVROS. Any MAVLink bridge must explicitly handle PX4/MAVLink frame conversion.
- `docs/marker.png` was validated as an upstream fractal marker, not a usable multi-ID nested marker.
- The corrected nested OpenCV ArUco test asset is `docs/nested_aruco_4x4_50.png`.

## Git Safety Rules

- Work in:

  ```bash
  cd ~/PX4/examples/gimbal_simulation
  ```

- Do not push from `~/PX4`, because that repository points to `PX4/PX4-Autopilot`.
- Do not commit build artifacts:

  ```text
  ros2_ws/build/
  ros2_ws/install/
  ros2_ws/log/
  __pycache__/
  *.pyc
  ```

- Before any future commit, check:

  ```bash
  git status --short --branch
  git diff --cached --name-status
  git rev-list --left-right --count origin/main...HEAD
  ```

## Implementation Order

```text
Step 0 marker validation
-> dib_msgs
-> detector
-> nested selector
-> transform
-> MAVLink bridge
-> flight integration
```

Do not implement the next component until the current component's test gate passes.

## Step 0: Validate Nested Marker

Goal: decide whether `docs/marker.png` can satisfy `REQ_PL_NEST_001`, `TEST 2.2`, and Task 2.4.

Status checkpoint before reboot:

- Original upstream marker `docs/marker.png`: FAIL for multi-ID nested switching.
- Reason: OpenCV `DICT_ARUCO_MIP_36h12` detects only stable ID `211`.
- Upstream repo uses `aruco.FractalDetector()` with `FRACTAL_5L_6`; that is a single fractal-marker pose detector, not an OpenCV multi-ID nested selector.
- Corrected centered multi-ID marker generated:

  ```text
  docs/nested_aruco_4x4_50.png
  docs/nested_aruco_4x4_50.json
  ```

- Corrected marker IDs:

  ```text
  outer_id=10, size=1.00m
  middle_id=11, size=0.22m
  inner_id=12, size=0.08m
  dictionary=DICT_4X4_50
  ```

- Offline validation PASS:

  ```bash
  cd ~/PX4/examples/gimbal_simulation
  python3 scripts/validate_nested_marker.py \
    --image docs/nested_aruco_4x4_50.png \
    --dictionary DICT_4X4_50
  ```

  ```text
  unique_ids: [10, 11, 12]
  stable_ids_count>=2: [10, 11, 12]
  RESULT: PASS - nested multi-ID detection is stable enough for selector testing.
  ```

- Gazebo assets created:

  ```text
  px4/Tools/simulation/gz/models/nested_aruco_marker/
  px4/Tools/simulation/gz/worlds/nested_aruco_landing.sdf
  ```

- Static checks passed:

  ```text
  XML parse model.sdf: PASS
  XML parse nested_aruco_landing.sdf: PASS
  texture hash docs <-> model: PASS
  detector validation on model texture: PASS
  ```

- Gazebo runtime test PASS:

  ```bash
  HOME=/tmp GZ_SIM_RESOURCE_PATH=~/PX4/examples/gimbal_simulation/px4/Tools/simulation/gz/models:~/PX4/Tools/simulation/gz/models \
    gz sim -s -r --iterations 20 \
    ~/PX4/examples/gimbal_simulation/px4/Tools/simulation/gz/worlds/nested_aruco_landing.sdf
  ```

- Gazebo camera detection test PASS at low altitude:

  ```text
  world: nested_aruco_landing
  model: gz_x500_gimbal
  marker pose moved to: (0, 0, 0)
  UAV pose during PASS: z ~= 1.50 m
  gimbal: manually pointed down through QGroundControl / gimbal control
  probe script: scripts/probe_gazebo_nested_aruco.py
  result: PASS
  detected IDs: [11, 12]
  frames: 164
  id_counts: {11: 73, 12: 88}
  pass_frames: 36
  best frame: /tmp/nested_aruco_step0_best_low.png
  ```

- Gazebo camera detection test FAIL at about 4 m:

  ```text
  UAV pose during FAIL: z ~= 3.94 m
  camera frame contained the marker, but OpenCV detected no IDs.
  likely cause: 1.0 m outer marker is too small/blurred at 4 m with the current gimbal camera render.
  action: use lower altitude for middle/inner detection, enlarge the simulated outer marker, or tune marker layout/rendering before expecting high-altitude nested detection.
  ```

Implementation:

- Run an offline OpenCV validation script.
- Load `docs/marker.png`.
- Test `DICT_ARUCO_MIP_36h12`.
- Test sensible full-image scales and centered crops because the image is very large.
- Report detected IDs, number of detections, rejected candidates, and whether nested multi-ID detection is stable.

Acceptance:

- PASS: at least two nested tag IDs are detected stably, suitable for outer -> inner switching.
- FAIL: only one tag ID is detected, or outer/inner detection is unstable.

Known observation before formal Step 0:

- `docs/marker.png` can detect as `DICT_ARUCO_MIP_36h12` ID `211` after downscale.
- Inner/middle IDs have not yet been proven detectable.

Original `docs/marker.png` decision:

- Do not continue to nested selector.
- Use the corrected nested marker instead.
- Keep the physical layout strategy: inner/middle tags centered inside outer tag, with separate valid IDs.

## Step 1: Create `dib_msgs`

Create package `dib_msgs` in `ros2_ws/src/dib_msgs`.

Status: PASS.

Build and interface checks completed:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dib_msgs
source install/setup.bash
ros2 interface show dib_msgs/msg/LandingTarget6D
ros2 interface show dib_msgs/msg/LandingTarget6DArray
```

`LandingTarget6D.msg`:

```text
uint8 LOST=0
uint8 SEARCHING=1
uint8 TRACKING=2

std_msgs/Header header
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
uint8 state
int32 tag_id
```

`LandingTarget6DArray.msg`:

```text
std_msgs/Header header
LandingTarget6D[] targets
```

Test gate:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dib_msgs
source install/setup.bash
ros2 interface show dib_msgs/msg/LandingTarget6D
ros2 interface show dib_msgs/msg/LandingTarget6DArray
```

## Step 2: Detector Node

Create `fiducial_detector_node` in `px4_offboard`.

Status: PASS.

Implemented:

- `ros2_ws/src/px4_offboard/px4_offboard/fiducial_detector_node.py`
- console entry point: `fiducial_detector_node`
- default dictionary: `DICT_4X4_50`
- default marker sizes:

  ```text
  id 10 -> 1.00m
  id 11 -> 0.22m
  id 12 -> 0.08m
  ```

Validation completed:

```bash
cd ~/PX4/examples/gimbal_simulation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select px4_offboard
```

Offline helper test on `docs/nested_aruco_4x4_50.png`:

```text
target_count: 3
ids: [10, 11, 12]
```

ROS 2 pub/sub test:

```text
received_target_count: 3
received_ids: [10, 11, 12]
```

Behavior:

- Subscribe `/gimbal_camera`.
- Detect all visible ArUco markers.
- Publish `/landing/targets_camera` as `dib_msgs/msg/LandingTarget6DArray`.
- Output frame is `camera_link`, OpenCV optical frame.
- Default dictionary is `DICT_4X4_50` for the corrected nested ArUco asset.

Parameters:

```text
dictionary
marker_size_m
image_topic
camera_frame
```

Requirements:

- `REQ_PL_VIDEO_001`
- `REQ_PL_DET_001`

Test gate:

- `ros2 topic hz /landing/targets_camera` is at least 20 Hz in simulation or sample playback.
- Detection rate is greater than 95% on the selected simulation/sample input.
- Output includes `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `state`, and `tag_id`.

## Step 3: Nested Selector Node

Create a node that subscribes `/landing/targets_camera` and publishes `/landing/selected_target_camera`.

Status: PASS.

Implemented:

- `ros2_ws/src/px4_offboard/px4_offboard/nested_target_selector_node.py`
- console entry point: `nested_target_selector_node`

Validation completed:

```text
unit sequence: far outer -> hold outer -> middle -> hold middle -> inner -> hold inner
actual IDs: [10, 10, 11, 11, 12, 12]
```

ROS 2 pub/sub test:

```text
selected_sequence: [10, 10, 11, 11, 12, 12]
```

Config:

```text
outer_tag_id
middle_tag_id
inner_tag_id
switch_in_z = 1.0
switch_out_z = 1.3
switch_stable_sec = 0.5
target_hold_sec = 0.5
```

Logic:

- Far range: choose outer tag.
- Near range: choose inner tag if it is detected and stable.
- Use distance hysteresis: `switch_in_z < switch_out_z`.
- Use time hysteresis: candidate must be stable for `0.5s`.
- If the inner tag is briefly lost for less than `0.5s`, hold the last selected target.
- If target loss exceeds `0.5s`, fall back to outer tag or publish `SEARCHING`.

Requirements:

- `REQ_PL_NEST_001`
- Task 2.4

Test gate, `TEST 2.2`:

- Dynamic input from far to near.
- Selected target switches outer -> inner.
- Total signal gap or strong coordinate disturbance during switching is less than `0.5s`.
- Selected target does not jump to an unrelated center.

## Step 4: Transform Node

Create a transform node that subscribes `/landing/selected_target_camera` and publishes `/landing/selected_target_body`.

Behavior:

- Input frame: `camera_link`, OpenCV optical frame.
- Output frame: `base_link`.
- Use a static transform for `camera_link -> base_link`.
- The MAVLink bridge later performs PX4/MAVLink frame conversion explicitly.

Requirements:

- `REQ_PL_COORD_001`
- `REQ_PL_COORD_002`

Test gate, `TEST 2.0`:

- Static transform error is less than `1cm`.
- Four physical sign checks pass 100%: left, right, forward, backward relative to the configured body frame.

## Step 5: MAVLink Bridge

Create a bridge that subscribes `/landing/selected_target_body` and sends MAVLink `LANDING_TARGET` through `pymavlink`.

Behavior:

- Publish target at 30 Hz; minimum acceptable rate is 20 Hz.
- If `state == LOST`, stop sending fresh coordinates.
- Do not send stale or fabricated target coordinates.
- Convert explicitly to the MAVLink/PX4 frame used in `landing_target_send`.

Requirements:

- `REQ_PL_CTRL_001`
- `REQ_PL_OUT_001`

Test gate:

- Bridge output rate is at least 20 Hz.
- PX4 receives and accepts target updates.
- LOST state does not produce stale target updates.

## Step 6: Flight Integration

Only start after Steps 0-5 pass.

Keep the proven PX4/QGround-safe flow:

```text
Offboard search/center/descend
-> NAV_LAND
-> PX4 land detector
-> normal disarm
```

Test gate:

- PX4 logs `Landing at current position`.
- PX4 logs `Landing detected`.
- PX4 logs `Disarmed by external command`.
- No default force-disarm path is used.

## Requirement Coverage

| Requirement | Covered by | Status |
| --- | --- | --- |
| `REQ_PL_VIDEO_001` | Detector rate/latency test | Simulation phase |
| `REQ_PL_CTRL_001` | MAVLink bridge rate test | Later step |
| `REQ_PL_LINK_001` | UART/Ethernet reliability | Hardware phase |
| `REQ_PL_RES_001` | Jetson CPU profiling | Hardware phase |
| `REQ_PL_DET_001` | Detector node 6DoF output | Step 2 |
| `REQ_PL_DET_002` | 0.5m-10m distance benchmark | Hardware phase |
| `REQ_PL_NEST_001` | Corrected marker + nested selector + TEST 2.2 | Step 0 asset PASS; Step 3 selector pending |
| `REQ_PL_COORD_001` | Transform node + TEST 2.0 | Step 4 |
| `REQ_PL_COORD_002` | Sign checks + TEST 2.0 | Step 4 |
| `REQ_PL_OUT_001` | MAVLink bridge acceptance | Step 5 |

## Assumptions

- No README edits are required for this implementation phase.
- No further git push happens unless explicitly requested.
- Hardware-only requirements are tracked but do not block simulation component development.
- `docs/marker.png` is retained as a reference fractal marker only.
- Use `docs/nested_aruco_4x4_50.png` for OpenCV nested ArUco implementation.
