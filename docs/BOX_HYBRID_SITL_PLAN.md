# Box-driven Hybrid Precision Landing SITL Plan

This document captures the staged plan for moving the current standalone
Fractal ArUco MAVROS landing demo toward a box-manager-style, mission-driven
landing flow.

## Target Runtime Shape

```text
PX4 SITL + Gazebo
MAVROS
box_manager
mock_box_hardware_node
mavros_to_dib_drone_telemetry_node
box_hybrid_precision_lander
aruco_fractal_tracker
```

The core rule is:

```text
The mission flies the UAV to the box region.
The lander does not use FLY_TO_SEARCH as the primary motion command.
```

`search_x/search_y` become a simulation fixture or fallback hold point, not the
normal way to fly to the box.

## Phase 1: Mission-driven Drone Flow

Current standalone demo:

```text
INIT -> TAKEOFF -> GIMBAL_DOWN -> FLY_TO_SEARCH -> SEARCH -> LANDING
```

Mission-integrated flow:

```text
IDLE
-> DRONE_MISSION
-> PRELANDING_CHECK
-> WAIT_BOX_READY
-> PX4_PRECISION_LANDING
-> DONE / FALLBACK
```

`IDLE` here is only the landing node standby state. It does not mean the full
UAV/box system is idle. In the real flow, the UAV may already be armed, in air,
and flying a QGround/PX4 mission while this lander remains in `IDLE` waiting for
the landing phase trigger.

Required behavior:

```text
1. Receive landing-phase trigger from one of the configured sources.
2. Bind mission target from box telemetry:
   box_info.box_id != 0
   box_info.latitude / box_info.longitude
   box_info.yaw
3. Let PX4 fly the mission using GPS/RTK toward that box target.
4. Trigger prelanding at the final waypoint or within a distance gate.
5. Prepare gimbal, camera, and tracker.
6. Request landing from the box.
7. Start visual landing only after box_state == WAITING_FOR_LANDING.
```

Trigger sources:

```text
trigger_mode=manual
  Manual/block-test mode. `/box_hybrid_landing/trigger` queues the transition.

trigger_mode=mission
  Mission-integrated mode. The node auto-triggers only after the UAV is in air,
  box telemetry is valid, and the mission reaches the final waypoint or the UAV
  enters the box arrival gate.

trigger_mode=both
  Debug/autonomy hybrid. Either manual trigger or mission arrival can start the
  landing phase.
```

This replaces the old `auto_start` idea. Node startup alone must never start
landing. Startup only puts the landing node into standby.

## Phase 2: Gazebo Box and Marker

Status: implemented as a static SITL fixture.

```text
dib_box_landing_pad
  base visual
  landing pad surface
  fractal marker plane/texture
```

Recommended initial pose:

```text
x = 4.0
y = -3.5
z = 0.0
yaw = 0.0
```

This can later move anywhere, as long as these agree:

```text
Gazebo box pose
box telemetry position
mission landing/prelanding waypoint
marker pose and yaw
```

For yaw testing, set a non-zero box yaw and rotate the marker texture/model with
the box. The acceptance check should compare UAV yaw against the box/marker yaw
near touchdown.

Current files:

```text
PX4 runtime:
  Tools/simulation/gz/models/dib_box_landing_pad/model.sdf
  Tools/simulation/gz/worlds/fractal_aruco_landing.sdf

Repo mirror for rsync:
  examples/gimbal_simulation/px4/Tools/simulation/gz/models/dib_box_landing_pad/model.sdf
  examples/gimbal_simulation/px4/Tools/simulation/gz/worlds/fractal_aruco_landing.sdf
```

The model reuses:

```text
model://fractal_aruco_marker/marker.png
```

so `fractal_aruco_marker` must still be synced alongside the box model.

## Phase 3: Mock Box Hardware

The real `box_manager` waits on hardware status. SITL needs a mock node for:

```text
/lid/cmd                  -> /lid/status
/clamp/cmd                -> /clamp/status
/dock/power_button/cmd
/dock/charge/cmd          -> /dock/charge/status
/dock/cooling_battery/cmd -> /dock/cooling_battery/status
/system1/power/status
gps
rtk_info
```

Mock behavior should be deterministic:

```text
lid open command    -> publish OPENED after a short delay
lid close command   -> publish CLOSED after a short delay
clamp open command  -> publish open positions
clamp close command -> publish configured close positions
```

This lets `box_manager` progress through:

```text
PREPARING_FOR_LANDING -> WAITING_FOR_LANDING
SECURING_DRONE -> CHARGING
```

## Phase 4: MAVROS to dib_msgs Telemetry Bridge

`box_manager` expects drone telemetry on:

```text
d<drone_id>/telemetry
```

Create a bridge from:

```text
/mavros/state
/mavros/extended_state
/mavros/battery
/mavros/local_position/pose
```

Minimum required output fields:

```text
header.stamp
state.connected
state.system_status
state.landed_state
```

Without this bridge, `box_manager` will not reliably transition from
`WAITING_FOR_LANDING` to `SECURING_DRONE`.

## Phase 5: New Hybrid Lander Node

Prefer a new node over modifying the existing standalone landers:

```text
box_hybrid_precision_lander
```

Reuse ideas from:

```text
precision_landing:
  mission upload
  box telemetry
  box command client

fractal_aruco_precision_lander:
  tracker target filtering
  visual guidance gates
  fallback behavior
  gimbal command
```

Main interfaces:

```text
Service:
  d<drone_id>/mission_upload

Subscribe:
  /b<box_id>/telemetry
  /mavros/mission/reached
  /mavros/mission/waypoints
  /mavros/state
  /mavros/extended_state
  /landing/target_camera

Client:
  /b<box_id>/cmd
  /mavros/set_mode

Publish:
  /mavros/landing_target/raw
  /mavros/setpoint_position/local  (OFFBOARD visual servo / SITL manual drive)
  /box_hybrid_landing/state
```

## Yaw Positioning

ArUco/fractal pose có orientation nên có thể làm yaw alignment, nhưng đây là
nhánh thử nghiệm. Mặc định `enable_yaw_setpoint:=false` để ưu tiên ổn định XY
và hạ cánh trước. Lý do: nếu yaw bị chỉnh liên tục theo từng frame marker,
camera sẽ xoay theo drone, target XY thay đổi mạnh và rất dễ rơi vào
`TARGET_LOST`.

Không dựa vào PX4 native precision landing để tự align yaw. Nếu cần yaw, node
hybrid phải tự giữ visual XY closed-loop trong lúc xoay.

Flow mặc định:

```text
Horizontal_approach
-> Descend_over_target
```

Flow khi bật thử nghiệm yaw:

```text
Horizontal_approach
-> Descend_over_target
-> Yaw_align_at_final_alt
-> Auto.land
```

Yaw alignment theo Option B: descend tới `final_alt`, giữ XY/altitude, chỉnh yaw
xong mới trigger `AUTO.LAND`. Không chỉnh yaw sau khi đã gửi `AUTO.LAND`.

```text
xy_error <= xy_gate
yaw_error <= yaw_gate for N frames
```

Suggested starting gates:

```text
xy_gate = 0.15 - 0.30 m
yaw_gate = 3 - 5 deg
final_alt = 1.0 m
```

Implementation hiện tại:

```text
1. Mọi setpoint giữ nguyên yaw hiện tại, không ép yaw về 0.
2. Nếu enable_yaw_setpoint=false: bỏ qua YAW_ALIGN.
3. DESCEND_OVER_TARGET ưu tiên XY và guarded descent tới final_alt.
4. Nếu enable_yaw_setpoint=true: tại final_alt, YAW_ALIGN giữ XY + altitude,
   yaw target lấy từ box_info.yaw.
5. Yaw align xong mới hand final phase to AUTO.LAND.
```

## SITL Test Ladder

1. Current standalone Fractal landing still works.
2. Gazebo box model appears and tracker detects marker on the box.
3. `box_manager` + `mock_box_hardware_node` reaches `WAITING_FOR_LANDING`.
4. Telemetry bridge drives `box_manager` landed transitions.
5. Hybrid lander receives mission and waits for final waypoint.
6. Hybrid lander requests landing and waits for box readiness.
7. Hybrid lander uses marker target for XY guidance.
8. Yaw alignment passes at a safe altitude.
9. Full landing completes and box transitions to securing.

## Fallback Rules

```text
Prelanding check fails
  -> fallback GPS/RTK landing or hold

Box does not reach WAITING_FOR_LANDING
  -> fallback GPS/RTK landing or hold

Marker search timeout
  -> fallback GPS/RTK landing

Target lost above commit altitude
  -> hold/reacquire/search

Target lost below commit altitude
  -> short hold, then AUTO.LAND

MAVROS disconnected
  -> stop visual guidance and let PX4 failsafe/AUTO.LAND behavior take over
```

## SITL Execution Runbook

To run the complete hybrid precision landing simulation end-to-end, open 4 separate terminal windows:

### Terminal 1: PX4 SITL & Gazebo World
```bash
cd ~/PX4
PX4_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal_fractal_aruco_landing
```

### Terminal 2: MAVROS Connection Bridge
```bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### Terminal 3: Unified Box & Landing Launch
Chạy toàn bộ cụm ROS 2 node: tracker, lander, telemetry bridge, mock box
hardware, box state manager, Gazebo bridges.

Mặc định launch dùng `trigger_mode=manual`, phù hợp để test từng block FSM:

```bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch px4_offboard box_hybrid_landing.launch.py
```

Manual trigger dưới đây mô phỏng sự kiện `mission_received + box_id`. Sau khi
nhận trigger, `box_hybrid_precision_lander` sẽ chờ `/b1/telemetry` và bind:

```text
box_info.box_id
box_info.latitude
box_info.longitude
box_info.yaw
```

`DRONE_MISSION` đại diện cho chặng mission GPS/RTK bay về vùng box. Trên hệ
thực, mission này có thể đến từ box scheduler, QGround, hoặc PX4 mission stack.
Trong `trigger_mode:=manual`, đây là chế độ test SITL: sau khi bind telemetry,
lander sẽ request `OFFBOARD` và publish local setpoint tới fixture box
`(x=4.0, y=-3.5, z=manual_drive_alt)`. `manual_drive_alt` mặc định là `10.0m`,
đóng vai trò độ cao approach/visual acquire ban đầu trong SITL. Khi UAV tới gần box, FSM mới vào
`PRELANDING_CHECK`. Trong các state chờ box/search/visual, lander vẫn stream
setpoint giữ vị trí để PX4 không báo `No offboard signal`, đồng thời tiếp tục
command gimbal nhìn xuống. Nếu muốn manual chỉ monitor mà không tự bay:

```bash
ros2 launch px4_offboard box_hybrid_landing.launch.py \
  trigger_mode:=manual manual_drive_to_box:=false
```

Trong `trigger_mode:=mission`, node không tự bay tới box; QGround/PX4 mission
phải bay UAV tới vùng box, còn lander chỉ theo dõi tiến độ waypoint MAVROS và
khoảng cách tới GPS target trong telemetry.

Nếu marker mất tạm thời sau khi đã acquire, FSM sẽ vào `TARGET_LOST`, giữ
position/gimbal down để thử reacquire trước khi quay lại `SEARCH`; không fallback
ngay ở frame mất đầu tiên.

Gửi trigger mô phỏng `mission_received + box_id`:
```bash
ros2 topic pub --once /box_hybrid_landing/trigger std_msgs/msg/String "data: 'land'"
```

### Mission trigger mode với QGround/PX4

Với `trigger_mode:=mission`, nên chạy lander trước khi bắt đầu bay mission, hoặc
ít nhất trước khi UAV tới waypoint cuối. Lý do là lander cần nghe:

```text
/mavros/mission/waypoints
/mavros/mission/reached
```

để biết mission có bao nhiêu waypoint và waypoint cuối đã được PX4 báo reached
hay chưa. QGround không cần publish trigger riêng cho node này. QGround chỉ cần:

```text
1. Upload/plan mission.
2. Start mission cho PX4 bay AUTO.MISSION.
3. PX4/MAVROS publish waypoint progress.
4. Lander tự detect final waypoint hoặc arrival gate rồi vào PRELANDING_CHECK.
```

Thứ tự test khuyến nghị:

```text
1. Start PX4 SITL + Gazebo.
2. Start MAVROS.
3. Start box_hybrid_landing.launch.py trigger_mode:=mission.
4. Trong QGround: upload mission, arm, start mission.
5. Theo dõi /box_hybrid_landing/state.
```

Lệnh chạy mission trigger mode:

```bash
ros2 launch px4_offboard box_hybrid_landing.launch.py trigger_mode:=mission
```

Trong mode này, node vẫn ở `IDLE` cho tới khi đủ toàn bộ guard của landing step:

```text
/mavros/state.connected == true
/b1/telemetry có box_id và GPS box hợp lệ
drone đang bay và cao hơn mission_trigger_min_alt
waypoint cuối đã reached HOẶC khoảng cách tới box <= arrival gate
```

Nếu lỡ mở lander sau khi UAV đã qua waypoint cuối, node có thể vẫn vào landing
nhờ GPS/local distance gate nếu UAV còn ở gần box. Tuy nhiên để test đúng logic
mission-integrated, nên mở lander trước mission để bắt được waypoint event sạch.

Để debug SITL linh hoạt, có thể dùng:

```bash
ros2 launch px4_offboard box_hybrid_landing.launch.py trigger_mode:=both
```

Mode `both` cho phép QGround mission arrival tự trigger, nhưng vẫn giữ manual
trigger để ép vào landing step sau khi đã kiểm tra telemetry.

### Terminal 4: Visual Camera Overlay (RQT)
```bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
(chọn annotated_image)
### Terminal 5 (Theo dõi FSM State của Drone):

```bash
  source /opt/ros/humble/setup.bash
  source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
  ros2 topic echo /box_hybrid_landing/state
```
Topic này do `box_hybrid_status_monitor` publish, nên luôn tồn tại khi launch
đang chạy. Nếu lander chưa chạy hoặc bị chết, topic sẽ hiện `LANDER_UNKNOWN`
hoặc `LANDER_STALE`. Khi lander hoạt động bình thường, dòng chảy kỳ vọng:
IDLE ➔ DRONE_MISSION ➔ PRELANDING_CHECK ➔ SEARCH ➔
HORIZONTAL_APPROACH ➔ DESCEND_OVER_TARGET ➔ LAND ➔ FLIGHT_IN_PROGRESS ➔ DONE.
Nếu launch với `enable_yaw_setpoint:=true`, sẽ có thêm bước thử nghiệm
`YAW_ALIGN` tại `final_alt`, trước `LAND`.

Nếu muốn cực sạch, chỉ publish khi state đổi:

```bash
ros2 launch px4_offboard box_hybrid_landing.launch.py state_heartbeat_sec:=0.0
```

### Terminal 6 (Theo dõi State & Command phản hồi của Box):

```bash
  source /opt/ros/humble/setup.bash
  source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
  ros2 topic echo /box_hybrid_landing/box_state
```
Topic này được publish bởi `box_hybrid_status_monitor`, độc lập với lander, nên
vẫn xem được box state nếu lander gặp lỗi. Topic publish heartbeat mặc định 1 Hz
và publish ngay khi box state đổi. Màn hình sẽ hiển thị các bước chuyển của Box:
EMPTY ➔ PREPARING_FOR_LANDING ➔ WAITING_FOR_LANDING ➔ SECURING_DRONE.

### Terminal 7 — Xem TOÀN BỘ telemetry đầy đủ của Box (gồm box_info, power, state, environment...):

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 topic echo /b1/telemetry
```

### Terminal 8 — Bắt log service call từ drone gửi đến box (BoxCmd REQUEST_LANDING):

Vì service call không echo được trực tiếp, cách tốt nhất là dùng ros2 service echo (ROS 2 Humble có hỗ trợ):

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 topic echo /box_hybrid_landing/comms
```

### Nếu `/box_hybrid_landing/state` hoặc `/box_hybrid_landing/comms` chưa xuất hiện

Thông báo kiểu này:

```text
WARNING: topic [/box_hybrid_landing/state] does not appear to be published yet
Could not determine the type for the passed topic
```

nghĩa là ROS graph hiện tại chưa thấy publisher của topic đó. Kiểm tra theo thứ
tự sau:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 node list | grep box_hybrid
ros2 topic list | grep box_hybrid
```

Kết quả đúng tối thiểu phải có:

```text
/box_hybrid_status_monitor
/box_hybrid_precision_lander
/box_hybrid_landing/state
/box_hybrid_landing/comms
```

Nếu không thấy `/box_hybrid_status_monitor`, launch đang chạy bản cũ hoặc package
chưa build/source lại. Nếu thấy monitor nhưng state là `LANDER_UNKNOWN` hoặc
`LANDER_STALE`, lúc đó mới kiểm tra lander. Có thể chạy riêng lander để kiểm tra
entry point:

```bash
ros2 run px4_offboard box_hybrid_precision_lander --ros-args -p use_sim_time:=false
```

Nếu chạy riêng lên log `Box hybrid precision lander ready...` thì executable ổn;
lỗi nằm ở launch/runtime dependency. Khi dùng launch SITL, nhớ chạy Gazebo/PX4 và
clock bridge trước hoặc cùng launch, vì node đang dùng `use_sim_time:=true`.
