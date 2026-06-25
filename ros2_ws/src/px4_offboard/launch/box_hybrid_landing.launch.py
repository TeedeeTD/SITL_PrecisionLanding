import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Start prelanding flow after MAVROS connects, useful before mission upload is integrated",
    )
    sim_box_ready_after_sec_arg = DeclareLaunchArgument(
        "sim_box_ready_after_sec",
        default_value="8.0",
        description="Delay before the simulated box publishes WAITING_FOR_LANDING",
    )
    enable_yaw_setpoint_arg = DeclareLaunchArgument(
        "enable_yaw_setpoint",
        default_value="false",
        description="Publish local yaw setpoints during YAW_ALIGN; keep false until mode ownership is tested",
    )

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
        output="screen",
    )

    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
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
        output="screen",
    )

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
        output="screen",
    )

    sim_box_node = Node(
        package="px4_offboard",
        executable="sim_box_manager",
        parameters=[
            {
                "state_topic": "/sim_box/state",
                "ready_after_sec": LaunchConfiguration("sim_box_ready_after_sec"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    hybrid_lander_node = Node(
        package="px4_offboard",
        executable="box_hybrid_precision_lander",
        parameters=[
            {
                "target_topic": "/landing/target_camera",
                "sim_box_state_topic": "/sim_box/state",
                "box_ready_state": "WAITING_FOR_LANDING",
                "auto_start": LaunchConfiguration("auto_start"),
                "marker_size": 0.50,
                "enable_yaw_setpoint": LaunchConfiguration("enable_yaw_setpoint"),
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
            auto_start_arg,
            sim_box_ready_after_sec_arg,
            enable_yaw_setpoint_arg,
            image_bridge_node,
            clock_bridge_node,
            camera_info_bridge_node,
            tracker_node,
            sim_box_node,
            hybrid_lander_node,
        ]
    )
