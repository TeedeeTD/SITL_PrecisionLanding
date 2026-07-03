import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Paths ---
    box_manager_config_path = PathJoinSubstitution([
        FindPackageShare('box_manager'),
        'config',
        'box_state_manager.yaml'
    ])

    marker_configuration_arg = DeclareLaunchArgument(
        "marker_configuration",
        default_value=os.path.join(
            os.path.expanduser("~"),
            "PX4/Tools/simulation/gz/models/fractal_aruco_marker/custom_fractal.yml",
        ),
        description="Absolute path to the custom fractal marker configuration",
    )
    camera_offset_x_arg = DeclareLaunchArgument(
        "camera_offset_x",
        default_value="0.1517",
        description="Camera physical X offset relative to drone center in body FLU frame",
    )
    camera_offset_y_arg = DeclareLaunchArgument(
        "camera_offset_y",
        default_value="0.0",
        description="Camera physical Y offset relative to drone center in body FLU frame",
    )
    enable_yaw_setpoint_arg = DeclareLaunchArgument(
        "enable_yaw_setpoint",
        default_value="true",
        description="Publish local yaw setpoints during YAW_ALIGN",
    )
    yaw_gate_deg_arg = DeclareLaunchArgument(
        "yaw_gate_deg",
        default_value="5.0",
        description="Yaw alignment tolerance in degrees before AUTO.LAND",
    )
    enable_offboard_visual_servo_arg = DeclareLaunchArgument(
        "enable_offboard_visual_servo",
        default_value="true",
        description="Use OFFBOARD setpoints for visual approach after mission arrival",
    )
    trigger_mode_arg = DeclareLaunchArgument(
        "trigger_mode",
        default_value="manual",
        description="Landing trigger source: manual, mission, or both",
    )
    state_heartbeat_sec_arg = DeclareLaunchArgument(
        "state_heartbeat_sec",
        default_value="1.0",
        description="/box_hybrid_landing/state heartbeat period; 0 publishes only on state changes",
    )
    box_state_heartbeat_sec_arg = DeclareLaunchArgument(
        "box_state_heartbeat_sec",
        default_value="1.0",
        description="/box_hybrid_landing/box_state heartbeat period; 0 publishes only on state changes",
    )
    manual_drive_to_box_arg = DeclareLaunchArgument(
        "manual_drive_to_box",
        default_value="true",
        description="In manual trigger mode, command OFFBOARD setpoints to the SITL box fixture",
    )
    manual_drive_alt_arg = DeclareLaunchArgument(
        "manual_drive_alt",
        default_value="10.0",
        description="Altitude used by manual SITL drive-to-box setpoint",
    )
    box_id_arg = DeclareLaunchArgument(
        "box_id",
        default_value="1",
        description="Box ID for namespaced telemetry and commands",
    )
    drone_id_arg = DeclareLaunchArgument(
        "drone_id",
        default_value="1",
        description="Drone ID for telemetry mapping",
    )

    # 1. Gazebo Bridges
    image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image"],
        remappings=[
            (
                "/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image",
                "/gimbal_camera",
            )
        ],
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="log",
    )

    camera_info_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[
            (
                "/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info",
                "/gimbal_camera/camera_info",
            )
        ],
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    # 2. ArUco Fractal Tracker Node
    tracker_node = Node(
        package="aruco_fractal_tracker",
        executable="aruco_fractal_tracker",
        parameters=[
            {
                "marker_configuration": LaunchConfiguration("marker_configuration"),
                "marker_size": 0.50,
                "min_tracking_z": 0.15,
                "max_tracking_z": 20.0,
                "max_pose_jump_m": 2.0,
                "acquire_good_frames": 8,
                "lost_bad_frames": 10,
                "show_latency_overlay": True,
                "latency_warn_ms": 100.0,
                "use_sim_time": True,
                "camera_x_to_body_east_sign": 1.0,
                "camera_y_to_body_north_sign": -1.0,
                "camera_offset_x": LaunchConfiguration("camera_offset_x"),
                "camera_offset_y": LaunchConfiguration("camera_offset_y"),
            }
        ],
        remappings=[
            ("image_input_topic", "/gimbal_camera"),
            ("camera_info_topic", "/gimbal_camera/camera_info"),
            ("image_output_topic", "/landing/annotated_image"),
            ("poses_output_topic", "/aruco_fractal_tracker/poses"),
            ("target_output_topic", "/landing/target_camera"),
        ],
        output="log",
    )

    # 3. Box Infrastructure Nodes
    mock_box_hardware_node = Node(
        package="px4_offboard",
        executable="mock_box_hardware",
        name="mock_box_hardware",
        parameters=[{"box_id": LaunchConfiguration("box_id")}],
        output="log",
    )

    mavros_to_dib_telemetry_node = Node(
        package="px4_offboard",
        executable="mavros_to_dib_telemetry",
        name="mavros_to_dib_telemetry",
        parameters=[{"drone_id": LaunchConfiguration("drone_id")}],
        output="log",
    )

    box_state_manager_node = Node(
        package="box_manager",
        executable="box_state_manager_node",
        name="box_state_manager",
        parameters=[
            box_manager_config_path,
            {"box_id": LaunchConfiguration("box_id")}
        ],
        output="log",
    )

    box_hybrid_status_monitor_node = Node(
        package="px4_offboard",
        executable="box_hybrid_status_monitor",
        name="box_hybrid_status_monitor",
        parameters=[
            {
                "box_id": LaunchConfiguration("box_id"),
                "box_state_heartbeat_sec": LaunchConfiguration("box_state_heartbeat_sec"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # 4. Hybrid Lander Node
    hybrid_lander_node = Node(
        package="px4_offboard",
        executable="box_hybrid_precision_lander",
        parameters=[
            {
                "target_topic": "/landing/target_camera",
                "box_id": LaunchConfiguration("box_id"),
                "drone_id": LaunchConfiguration("drone_id"),
                "marker_size": 0.50,
                "enable_yaw_setpoint": LaunchConfiguration("enable_yaw_setpoint"),
                "yaw_gate_deg": LaunchConfiguration("yaw_gate_deg"),
                "enable_offboard_visual_servo": LaunchConfiguration("enable_offboard_visual_servo"),
                "trigger_mode": LaunchConfiguration("trigger_mode"),
                "state_heartbeat_sec": LaunchConfiguration("state_heartbeat_sec"),
                "manual_drive_to_box": LaunchConfiguration("manual_drive_to_box"),
                "manual_drive_alt": LaunchConfiguration("manual_drive_alt"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            marker_configuration_arg,
            camera_offset_x_arg,
            camera_offset_y_arg,
            enable_yaw_setpoint_arg,
            yaw_gate_deg_arg,
            enable_offboard_visual_servo_arg,
            trigger_mode_arg,
            state_heartbeat_sec_arg,
            box_state_heartbeat_sec_arg,
            manual_drive_to_box_arg,
            manual_drive_alt_arg,
            box_id_arg,
            drone_id_arg,
            image_bridge_node,
            clock_bridge_node,
            camera_info_bridge_node,
            tracker_node,
            mock_box_hardware_node,
            mavros_to_dib_telemetry_node,
            box_state_manager_node,
            box_hybrid_status_monitor_node,
            hybrid_lander_node,
        ]
    )
