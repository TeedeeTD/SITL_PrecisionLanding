# Step 0 Marker Validation

## Goal

Validate whether `docs/marker.png` can satisfy nested target selection for:

- `REQ_PL_NEST_001`
- `TEST 2.2`
- Task 2.4

## Command

```bash
cd ~/PX4/examples/gimbal_simulation
python3 scripts/validate_nested_marker.py
```

## Result

```text
image: /home/ducanh/PX4/examples/gimbal_simulation/docs/marker.png
shape: 10800x10800
dictionary: DICT_ARUCO_MIP_36h12

full scale=0.1: ids=none rejected=49
full scale=0.075: ids=[211] rejected=40
full scale=0.05: ids=[211] rejected=27
center crop=0.7 resized=900: ids=[211] rejected=47
center crop=0.5 resized=900: ids=none rejected=67
center crop=0.35 resized=900: ids=none rejected=64
center crop=0.25 resized=900: ids=none rejected=71
center crop=0.18 resized=900: ids=none rejected=55
center crop=0.12 resized=900: ids=none rejected=40
center crop=0.08 resized=900: ids=none rejected=24

unique_ids: [211]
stable_ids_count>=2: [211]
id 211: seen 3 time(s) in ['full scale=0.075', 'full scale=0.05', 'center crop=0.7 resized=900']

RESULT: FAIL - fewer than two stable marker IDs were detected.
ACTION: choose or generate a corrected nested marker with separate valid IDs.
```

## Decision

`docs/marker.png` is not sufficient for nested target switching because it only
provides one stable marker ID. Do not continue to the nested selector step with
this marker.

## Next Action

Generate or choose a corrected nested marker layout where the outer and inner
markers are separate valid IDs. Keep the physical layout strategy where the
inner target is centered relative to the outer target, but do not destroy the
outer marker's encoded cells.

## Corrected Nested ArUco Candidate

The upstream `uav_landing_sim` marker is a single fractal marker, so it is not
used for multi-ID selector testing. A corrected centered OpenCV ArUco candidate
was generated instead:

- image: `docs/nested_aruco_4x4_50.png`
- metadata: `docs/nested_aruco_4x4_50.json`
- dictionary: `DICT_4X4_50`
- outer ID: `10`, size: `1.00 m`
- middle ID: `11`, size: `0.22 m`
- inner ID: `12`, size: `0.08 m`
- all marker centers are aligned with the landing point

Generation command:

```bash
cd ~/PX4/examples/gimbal_simulation
python3 scripts/generate_nested_aruco_marker.py
```

Validation command:

```bash
cd ~/PX4/examples/gimbal_simulation
python3 scripts/validate_nested_marker.py \
  --image docs/nested_aruco_4x4_50.png \
  --dictionary DICT_4X4_50
```

Result:

```text
unique_ids: [10, 11, 12]
stable_ids_count>=2: [10, 11, 12]
id 10: seen 3 time(s)
id 11: seen 7 time(s)
id 12: seen 10 time(s)

RESULT: PASS - nested multi-ID detection is stable enough for selector testing.
```

Gazebo assets were also created for the corrected marker:

- `px4/Tools/simulation/gz/models/nested_aruco_marker/`
- `px4/Tools/simulation/gz/worlds/nested_aruco_landing.sdf`

Static checks passed:

- `model.sdf` XML parse: PASS
- `nested_aruco_landing.sdf` XML parse: PASS
- model texture hash matches `docs/nested_aruco_4x4_50.png`: PASS
- detector validation on model texture: PASS

Gazebo runtime spawn/load test passed:

```bash
HOME=/tmp GZ_SIM_RESOURCE_PATH=~/PX4/examples/gimbal_simulation/px4/Tools/simulation/gz/models:~/PX4/Tools/simulation/gz/models \
  gz sim -s -r --iterations 20 \
  ~/PX4/examples/gimbal_simulation/px4/Tools/simulation/gz/worlds/nested_aruco_landing.sdf
```

## Gazebo Camera Detection Test

After spawning `gz_x500_gimbal` in `nested_aruco_landing`, the marker was moved
under the vehicle at `(0, 0, 0)`. The UAV was commanded to take off and the
gimbal camera was pointed down through QGroundControl.

At about `4 m` altitude, the camera frame contained the marker, but OpenCV did
not detect any nested IDs. The marker was too small and softened by rendering at
that distance for the current `1.0 m` physical marker size.

At about `1.5 m` altitude, the live Gazebo camera probe passed:

```bash
cd ~/PX4/examples/gimbal_simulation
HOME=/tmp ROS_LOG_DIR=/tmp/ros_log \
python3 scripts/probe_gazebo_nested_aruco.py \
  --topic /gimbal_camera \
  --dictionary DICT_4X4_50 \
  --expected-ids 10,11,12 \
  --duration 6 \
  --pass-min-ids 2 \
  --pass-min-frames 3 \
  --save-frame /tmp/nested_aruco_step0_best_low.png \
  --save-last-frame /tmp/nested_aruco_step0_last_low.png
```

Result:

```text
frames: 164
id_counts: {11: 73, 12: 88}
frames_with_expected: 125
pass_frames: 36
best_ids: [12, 11]
PROBE_RESULT PASS nested marker visible with multiple expected IDs
```

Decision:

- Step 0 Gazebo camera detection is valid at close range.
- The current marker layout/size is not sufficient for reliable high-altitude
  detection at about `4 m`.
- For selector design, use the outer marker for farther approach and switch to
  middle/inner markers only when the camera is close enough; alternatively
  enlarge the physical marker or generate a less compact nested layout for
  higher-altitude tests.
