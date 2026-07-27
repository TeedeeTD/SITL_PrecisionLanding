import sys
import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'common'
import json
import threading
import time
from urllib.parse import urlparse, parse_qs
from http.server import SimpleHTTPRequestHandler, HTTPServer
from pymavlink import mavutil

# State of the drone
drone_state = {
    "armed": False,
    "nav_state": 2, # default to POSCTL
    "action_str": "None",
    "logs": [],
}

# Complete mapping from HTML checkbox/dropdown IDs to MAVLink failure units & parameters
# Units:
# - gyro: 0, accel: 1, mag: 2, baro: 3, gps: 4, flow: 5, vio: 6, dist: 7, airspeed: 8
# - battery: 100, motor: 101, servo: 102, avoidance: 103, rc_signal: 104, mavlink_signal: 105
failure_mapping = {
    # Mode requirements / Sensors
    "angular_velocity_invalid": (0, 1),
    "attitude_invalid": (0, 1),
    "local_altitude_invalid": (3, 1),
    "local_position_invalid": (4, 1),
    "local_position_invalid_relaxed": (4, 1),
    "local_velocity_invalid": (4, 1),
    "global_position_invalid": (4, 1),
    "global_position_invalid_relaxed": (4, 1),
    "position_accuracy_low": (4, 4),
    "local_position_accuracy_low": (4, 4),
    "auto_mission_missing": (200, 1),
    "offboard_control_signal_lost": (201, 1),
    "home_position_invalid": (202, 1),

    # Control links
    "manual_control_signal_lost": (104, 1),
    "gcs_connection_lost": (105, 1),

    # Battery
    "battery_unhealthy": (100, 1),
    "battery_low_remaining_time": (100, 1),

    # Failure Detector
    "fd_critical_failure": (300, 1),
    "fd_esc_arming_failure": (301, 1),
    "fd_imbalanced_prop": (302, 1),
    "fd_motor_failure": (101, 1),

    # Other
    "geofence_breached": (400, 1),
    "mission_failure": (200, 1),
    "vtol_fixed_wing_system_failure": (401, 1),
    "wind_limit_exceeded": (402, 1),
    "flight_time_limit_exceeded": (403, 1),
    "navigator_failure": (404, 1),
}

# Global MAVLink connection
mav_conn = None
rc_loss_active = False

def rc_heartbeat_thread_func():
    global rc_loss_active
    while True:
        if not rc_loss_active:
            conn = get_mavlink_connection()
            if conn:
                try:
                    target_sys = conn.target_system if conn.target_system != 0 else 1
                    # Send neutral joystick setpoint (x=0, y=0, z=500, r=0) at 5Hz to tell PX4 RC is present
                    conn.mav.manual_control_send(target_sys, 0, 0, 500, 0, 0)
                except Exception:
                    pass
        time.sleep(0.2)

def get_mavlink_connection():
    global mav_conn
    if mav_conn is None:
        try:
            # Connect to SITL (UDP port 14540)
            mav_conn = mavutil.mavlink_connection('udp:127.0.0.1:14540')
            print("[MAVLink] Connected to 127.0.0.1:14540")
        except Exception as e:
            print("[MAVLink] Connection failed:", e)
    return mav_conn

def custom_mode_to_nav_state(custom_mode):
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    
    if main_mode == 1:   # MANUAL
        return 0
    elif main_mode == 2: # ALTCTL
        return 1
    elif main_mode == 3: # POSCTL
        return 2
    elif main_mode == 4: # AUTO
        if sub_mode == 2: return 17   # TAKEOFF
        elif sub_mode == 3: return 4  # LOITER / HOLD
        elif sub_mode == 4: return 3  # MISSION
        elif sub_mode == 5: return 5  # RTL
        elif sub_mode == 6: return 18 # LAND
        elif sub_mode == 8: return 19 # FOLLOW
        elif sub_mode == 9: return 20 # PRECLAND
    elif main_mode == 5: # ACRO
        return 10
    elif main_mode == 6: # OFFBOARD
        return 14
    elif main_mode == 7: # STABILIZED
        return 15
    return 2

def nav_state_to_custom_mode(nav_state):
    if nav_state == 0: return 1, 0      # MANUAL
    elif nav_state == 1: return 2, 0    # ALTCTL
    elif nav_state == 2: return 3, 0    # POSCTL
    elif nav_state == 3: return 4, 4    # AUTO_MISSION
    elif nav_state == 4: return 4, 3    # AUTO_LOITER (HOLD)
    elif nav_state == 5: return 4, 5    # AUTO_RTL
    elif nav_state == 10: return 5, 0   # ACRO
    elif nav_state == 14: return 6, 0   # OFFBOARD
    elif nav_state == 15: return 7, 0   # STABILIZED
    elif nav_state == 17: return 4, 2   # AUTO_TAKEOFF
    elif nav_state == 18: return 4, 6   # AUTO_LAND
    elif nav_state == 19: return 4, 8   # AUTO_FOLLOW
    elif nav_state == 20: return 4, 9   # AUTO_PRECLAND
    return 3, 0

def mavlink_thread_func():
    global drone_state
    while True:
        conn = get_mavlink_connection()
        if conn is None:
            time.sleep(1)
            continue
        try:
            msg = conn.recv_match(blocking=True, timeout=0.5)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'HEARTBEAT':
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    drone_state["armed"] = armed
                    drone_state["nav_state"] = custom_mode_to_nav_state(msg.custom_mode)
                elif msg_type == 'STATUSTEXT':
                    text = msg.text
                    log_line = f"User notification: {text}"
                    if not drone_state["logs"] or drone_state["logs"][-1] != log_line:
                        drone_state["logs"].append(log_line)
                        if len(drone_state["logs"]) > 50:
                            drone_state["logs"].pop(0)
                    if "failsafe" in text.lower() or "critical" in text.lower() or "landing" in text.lower() or "rtl" in text.lower() or "hold" in text.lower():
                        drone_state["action_str"] = text
        except Exception as e:
            print("[MAVLink Thread] Error:", e)
            time.sleep(1)

class LinkedFailsafeHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        url_parts = urlparse(self.path)
        path = url_parts.path
        query = parse_qs(url_parts.query)

        # Handle API calls
        if path == "/api/get_status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(drone_state).encode("utf-8"))
            return
            
        elif path == "/api/set_armed":
            val = query.get("value", ["false"])[0] == "true"
            conn = get_mavlink_connection()
            if conn:
                # 400 = MAV_CMD_COMPONENT_ARM_DISARM
                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1 if val else 0, 0, 0, 0, 0, 0, 0
                )
                print(f"[MAVLink] Sent arm command: {val}")
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            return
            
        elif path == "/api/set_mode":
            nav_state = int(query.get("value", [2])[0])
            main_mode, sub_mode = nav_state_to_custom_mode(nav_state)
            conn = get_mavlink_connection()
            if conn:
                # 176 = MAV_CMD_DO_SET_MODE
                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    main_mode, sub_mode, 0, 0, 0, 0
                )
                print(f"[MAVLink] Sent set mode command: main={main_mode}, sub={sub_mode}")
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            return
            
        elif path == "/api/inject_failure":
            global rc_loss_active
            el_id = query.get("id", [""])[0]
            val = query.get("value", ["false"])[0]
            
            if el_id == "manual_control_signal_lost":
                rc_loss_active = (val == "true")
                print(f"[MAVLink] RC loss state set to: {rc_loss_active}")

            unit = None
            type_val = 0 # OK
            
            if el_id == "battery_warning":
                unit = 100 # Battery
                # Map select value: 0=OK, 1=Low, 2=Critical, 3=Emergency
                type_val = int(val)
                if type_val > 0:
                    type_val = int(val) # Failure level
            elif el_id in failure_mapping:
                unit, type_val_mapped = failure_mapping[el_id]
                type_val = type_val_mapped if val == "true" else 0
                
            if unit is not None or el_id in failure_mapping or el_id == "battery_warning":
                disp_unit = unit if unit is not None else 100
                state_str = f"failed (type={type_val}), adding action" if (val in ("true", "1", "2", "3") or type_val > 0) else "valid, removing action"
                drone_state["logs"].append(f"Caller {disp_unit}: state changed to {state_str}")
                if len(drone_state["logs"]) > 50:
                    drone_state["logs"].pop(0)
                conn = get_mavlink_connection()
                if conn:
                    # Send MAVLink failure injection command (420)
                    if unit is not None and unit < 200:
                        cmd_inject_failure = getattr(mavutil.mavlink, 'MAV_CMD_INJECT_FAILURE', 420)
                        conn.mav.command_long_send(
                            conn.target_system, conn.target_component,
                            cmd_inject_failure,
                            0, unit, type_val, 0, 0, 0, 0, 0
                        )
                        print(f"[MAVLink] Sent inject failure: unit={unit}, type={type_val}")
                    
                    # Execute active Failsafe action in PX4 SITL
                    is_active = (val == "true" or (el_id == "battery_warning" and int(val) > 0))
                    if is_active:
                        # RTL Group
                        if el_id in ("manual_control_signal_lost", "gcs_connection_lost", "geofence_breached",
                                     "mission_failure", "vtol_fixed_wing_system_failure", "wind_limit_exceeded",
                                     "flight_time_limit_exceeded", "battery_low_remaining_time", "auto_mission_missing") or \
                           (el_id == "battery_warning" and val in ("1", "2")):
                            # 176 = MAV_CMD_DO_SET_MODE (main=4 AUTO, sub=5 RTL)
                            conn.mav.command_long_send(
                                conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                4, 5, 0, 0, 0, 0
                            )
                            print("[MAVLink] Triggered Failsafe Action -> AUTO_RTL")
                        
                        # LAND Group
                        elif el_id in ("global_position_invalid", "global_position_invalid_relaxed",
                                       "local_position_invalid", "local_position_invalid_relaxed",
                                       "local_velocity_invalid", "local_altitude_invalid",
                                       "home_position_invalid", "navigator_failure", "position_accuracy_low") or \
                             (el_id == "battery_warning" and val == "3"):
                            # 176 = MAV_CMD_DO_SET_MODE (main=4 AUTO, sub=6 LAND)
                            conn.mav.command_long_send(
                                conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                4, 6, 0, 0, 0, 0
                            )
                            print("[MAVLink] Triggered Failsafe Action -> AUTO_LAND")

                        # HOLD Group
                        elif el_id in ("local_position_accuracy_low", "offboard_control_signal_lost"):
                            # 176 = MAV_CMD_DO_SET_MODE (main=4 AUTO, sub=3 LOITER)
                            conn.mav.command_long_send(
                                conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                4, 3, 0, 0, 0, 0
                            )
                            print("[MAVLink] Triggered Failsafe Action -> AUTO_LOITER (HOLD)")

                        # DISARM / TERMINATE Group
                        elif el_id in ("fd_critical_failure", "fd_esc_arming_failure", "battery_unhealthy"):
                            # 400 = MAV_CMD_COMPONENT_ARM_DISARM (0=Disarm)
                            conn.mav.command_long_send(
                                conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 0, 0, 0, 0, 0, 0, 0
                            )
                            print("[MAVLink] Triggered Failsafe Action -> DISARM")
            
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            return
            
        elif path == "/failsafe_bridge.js":
            self.send_response(200)
            self.send_header("Content-Type", "application/javascript")
            self.end_headers()
            js_content = """
            console.log("Failsafe Bridge JS loaded!");
            
            // Wait for Emscripten Module to be defined and override update loops
            function setupOverrides() {
                if (typeof Module !== "undefined" && Module.onRuntimeInitialized) {
                    var originalRuntimeInit = Module.onRuntimeInitialized;
                    Module.onRuntimeInitialized = function() {
                        if (originalRuntimeInit) originalRuntimeInit();
                        
                        Module.failsafe_update = function(armed, vtol, mission, override, intended_mode, vtype, state, defer) {
                            return parseInt(document.querySelector('select[id="intended_nav_state"]').value);
                        };
                        
                        Module.selected_action = function() {
                            return window.currentFailsafeActive ? 1 : 0;
                        };
                        
                        Module.action_str = function(action) {
                            return window.currentFailsafeAction || "None";
                        };
                    };
                } else {
                    setTimeout(setupOverrides, 50);
                }
            }
            setupOverrides();
            
            // Poll status from backend
            window.currentFailsafeAction = "None";
            window.currentFailsafeActive = false;
            function pollDroneStatus() {
                fetch("/api/get_status")
                    .then(r => r.json())
                    .then(data => {
                        // Update armed status if element exists
                        var armedEl = document.getElementById("armed");
                        if (armedEl && document.activeElement !== armedEl) {
                            armedEl.checked = data.armed;
                        }
                        
                        // Update intended navigation state if dropdown exists
                        var modeEl = document.getElementById("intended_nav_state");
                        if (modeEl && document.activeElement !== modeEl) {
                            modeEl.value = data.nav_state;
                        }
                        
                        // Determine active failsafe action string
                        var rcLost = document.querySelector('input[id="manual_control_signal_lost"]')?.checked;
                        var gcsLost = document.querySelector('input[id="gcs_connection_lost"]')?.checked;
                        var geofence = document.querySelector('input[id="geofence_breached"]')?.checked;
                        var missionFail = document.querySelector('input[id="mission_failure"]')?.checked || document.querySelector('input[id="auto_mission_missing"]')?.checked;
                        var windExceeded = document.querySelector('input[id="wind_limit_exceeded"]')?.checked;
                        var flightTimeExceeded = document.querySelector('input[id="flight_time_limit_exceeded"]')?.checked;
                        var vtolFail = document.querySelector('input[id="vtol_fixed_wing_system_failure"]')?.checked;
                        var batLowTime = document.querySelector('input[id="battery_low_remaining_time"]')?.checked;
                        var batWarnVal = document.querySelector('select[id="battery_warning"]')?.value;
                        
                        var gpsInvalid = document.querySelector('input[id="global_position_invalid"]')?.checked ||
                                         document.querySelector('input[id="global_position_invalid_relaxed"]')?.checked ||
                                         document.querySelector('input[id="local_position_invalid"]')?.checked ||
                                         document.querySelector('input[id="local_position_invalid_relaxed"]')?.checked ||
                                         document.querySelector('input[id="local_velocity_invalid"]')?.checked ||
                                         document.querySelector('input[id="local_altitude_invalid"]')?.checked ||
                                         document.querySelector('input[id="home_position_invalid"]')?.checked ||
                                         document.querySelector('input[id="navigator_failure"]')?.checked;
                                         
                        var motorFail = document.querySelector('input[id="fd_motor_failure"]')?.checked;
                        var criticalFail = document.querySelector('input[id="fd_critical_failure"]')?.checked ||
                                           document.querySelector('input[id="fd_esc_arming_failure"]')?.checked ||
                                           document.querySelector('input[id="battery_unhealthy"]')?.checked;
                        
                        var rawText = data.action_str || "None";
                        var actionText = "None";
                        
                        var lower = rawText.toLowerCase();
                        if (criticalFail) {
                            actionText = "Disarm";
                        } else if (motorFail) {
                            actionText = "Terminate";
                        } else if (lower.includes("return") || lower.includes("rtl") || data.nav_state === 5 ||
                                   rcLost || gcsLost || geofence || missionFail || windExceeded || flightTimeExceeded || vtolFail || batLowTime || batWarnVal === "1" || batWarnVal === "2") {
                            actionText = "RTL";
                        } else if (lower.includes("land") || data.nav_state === 18 || gpsInvalid || batWarnVal === "3") {
                            actionText = "Land";
                        } else if (lower.includes("hold") || data.nav_state === 4 || document.querySelector('input[id="offboard_control_signal_lost"]')?.checked || document.querySelector('input[id="position_accuracy_low"]')?.checked) {
                            actionText = "Hold";
                        } else if (rawText !== "None") {
                            actionText = rawText;
                        }
                        
                        window.currentFailsafeAction = actionText;
                        window.currentFailsafeActive = (actionText !== "None");
                        
                        var actionEl = document.getElementById("action");
                        if (actionEl) {
                            if (actionText !== "None") {
                                actionEl.innerHTML = "<font color='crimson'>" + actionText + "</font>";
                            } else {
                                actionEl.innerHTML = "None";
                            }
                        }
                        
                        // Update debug output console textarea
                        var outputEl = document.getElementById("output");
                        if (outputEl && data.logs) {
                            var textContent = data.logs.join("\\n");
                            if (outputEl.value !== textContent) {
                                outputEl.value = textContent;
                                outputEl.scrollTop = outputEl.scrollHeight;
                            }
                        }
                    })
                    .catch(err => console.error("Error polling status:", err));
            }
            
            setInterval(pollDroneStatus, 500);
            
            // Watch ALL checkboxes and dropdown changes
            window.addEventListener("load", function() {
                var elementsToWatch = [
                    "angular_velocity_invalid", "attitude_invalid", "local_altitude_invalid",
                    "local_position_invalid", "local_position_invalid_relaxed", "local_velocity_invalid",
                    "global_position_invalid", "global_position_invalid_relaxed", "position_accuracy_low",
                    "local_position_accuracy_low", "auto_mission_missing", "offboard_control_signal_lost",
                    "home_position_invalid", "manual_control_signal_lost", "gcs_connection_lost",
                    "battery_unhealthy", "battery_low_remaining_time", "battery_warning",
                    "fd_critical_failure", "fd_esc_arming_failure", "fd_imbalanced_prop", "fd_motor_failure",
                    "geofence_breached", "mission_failure", "vtol_fixed_wing_system_failure",
                    "wind_limit_exceeded", "flight_time_limit_exceeded", "navigator_failure"
                ];
                
                elementsToWatch.forEach(function(id) {
                    var el = document.getElementById(id);
                    if (!el) return;
                    el.addEventListener("change", function() {
                        var val = el.type === "checkbox" ? el.checked : el.value;
                        fetch("/api/inject_failure?id=" + encodeURIComponent(id) + "&value=" + encodeURIComponent(val));
                    });
                });
                
                // Watch Armed and Mode changes to send MAVLink commands
                var armedEl = document.getElementById("armed");
                if (armedEl) {
                    armedEl.addEventListener("click", function() {
                        setTimeout(() => {
                            fetch("/api/set_armed?value=" + armedEl.checked);
                        }, 50);
                    });
                }
                
                var modeEl = document.getElementById("intended_nav_state");
                if (modeEl) {
                    modeEl.addEventListener("change", function() {
                        fetch("/api/set_mode?value=" + modeEl.value);
                    });
                }
            });
            """
            self.wfile.write(js_content.encode("utf-8"))
            return

        # Intercept and inject index.html
        if path == "/" or path == "/index.html":
            try:
                with open("index.html", "r", encoding="utf-8") as f:
                    content = f.read()
                # Inject failsafe_bridge.js script tag before </body>
                injected_content = content.replace("</body>", '<script src="failsafe_bridge.js"></script></body>')
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(injected_content.encode("utf-8"))
                return
            except Exception as e:
                self.send_error(500, f"Error reading index.html: {e}")
                return

        # Fallback to SimpleHTTPRequestHandler
        super().do_GET()

if __name__ == "__main__":
    # Start MAVLink connection thread
    t = threading.Thread(target=mavlink_thread_func, daemon=True)
    t.start()
    
    # Start RC heartbeat thread to keep RC active in PX4 until RC Loss is toggled
    t_rc = threading.Thread(target=rc_heartbeat_thread_func, daemon=True)
    t_rc.start()
    
    server_address = ('', 8000)
    httpd = HTTPServer(server_address, LinkedFailsafeHandler)
    print("Serving LINKED failsafe simulation on port 8000...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    print("Server stopped.")
