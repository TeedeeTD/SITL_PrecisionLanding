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

Required behavior:

```text
1. Receive mission.
2. Parse box_id and landing waypoint.
3. Let PX4 fly the mission using GPS/RTK.
4. Trigger prelanding at the final waypoint or within a distance gate.
5. Prepare gimbal, camera, and tracker.
6. Request landing from the box.
7. Start visual landing only after box_state == WAITING_FOR_LANDING.
```

## Phase 2: Gazebo Box and Marker

Create a static Gazebo model first:

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
  /mavros/setpoint_position/local  (only when yaw/offboard control is enabled)
  /box_hybrid_landing/state
```

## Yaw Positioning

ArUco/fractal pose includes marker orientation, so yaw alignment is possible.
Do not rely on PX4 native precision landing to align yaw.

Initial yaw plan:

```text
Horizontal_approach
-> Yaw_align
-> Descend_over_target
```

Yaw alignment should run at a safe altitude first:

```text
xy_error <= xy_gate
yaw_error <= yaw_gate for N frames
```

Suggested starting gates:

```text
xy_gate = 0.15 - 0.30 m
yaw_gate = 5 - 10 deg
final_alt = 1.0 m
```

Implementation path:

```text
1. Publish position setpoint with yaw through /mavros/setpoint_position/local.
2. Hold XY and altitude while yaw aligns.
3. Descend while holding XY and yaw.
4. Hand final phase to AUTO.LAND at final_alt.
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
