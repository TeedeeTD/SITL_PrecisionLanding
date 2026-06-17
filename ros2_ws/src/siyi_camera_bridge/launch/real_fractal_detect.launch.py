"""
Launch file: Real SIYI A8 Mini + Fractal ArUco Tracker + MAVROS

Pipeline:
  1. MAVROS — connects to FCU (optional, can skip if no FCU connected)
  2. siyi_camera_bridge — RTSP stream → /siyi/image_raw + /siyi/camera_info
  3. aruco_fractal_tracker — detect fractal marker, publish pose + debug image

Usage (camera only, no MAVROS):
  ros2 launch siyi_camera_bridge real_fractal_detect.launch.py enable_mavros:=false

Usage (camera + MAVROS):
  ros2 launch siyi_camera_bridge real_fractal_detect.launch.py enable_mavros:=true fcu_url:=/dev/ttyACM0:57600
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _launch_bool(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def _maybe_start_mavros(context):
    if not _launch_bool(LaunchConfiguration('enable_mavros').perform(context)):
        return []

    mavros_dir = get_package_share_directory('mavros')
    return [
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'px4.launch')
            ),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
            }.items(),
        )
    ]


def generate_launch_description():

    # ── Launch Arguments ────────────────────────────────────────────

    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://192.168.168.14:8554/main.264',
        description='RTSP stream URL of SIYI A8 Mini camera'
    )

    flip_180_arg = DeclareLaunchArgument(
        'flip_180',
        default_value='true',
        description='Flip image 180 degrees (camera mounted upside-down)'
    )

    marker_configuration_arg = DeclareLaunchArgument(
        'marker_configuration',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'PX4/examples/gimbal_simulation/px4/Tools/simulation/gz/models/'
            'fractal_aruco_marker/custom_fractal.yml'
        ),
        description='Absolute path to the fractal marker configuration YAML'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.50',
        description='Physical size of outer marker in meters'
    )

    enable_mavros_arg = DeclareLaunchArgument(
        'enable_mavros',
        default_value='true',
        description='Enable MAVROS node (set false if no FCU connected)'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14580',
        description='MAVROS FCU URL (e.g. /dev/ttyACM0:57600 for USB Pixhawk)'
    )

    # ── 1. MAVROS (optional) ────────────────────────────────────────

    mavros_launch = OpaqueFunction(function=_maybe_start_mavros)

    # ── 2. SIYI RTSP Camera Bridge ─────────────────────────────────

    rtsp_node = Node(
        package='siyi_camera_bridge',
        executable='rtsp_publisher',
        name='siyi_rtsp_publisher',
        parameters=[{
            'rtsp_url': LaunchConfiguration('rtsp_url'),
            'frame_id': 'siyi_camera_optical_frame',
            'flip_180': LaunchConfiguration('flip_180'),
            'target_fps': 30.0,
            'image_width': 1280,
            'image_height': 720,
        }],
        output='screen'
    )

    # ── 3. Fractal ArUco Tracker ────────────────────────────────────

    tracker_node = Node(
        package='aruco_fractal_tracker',
        executable='aruco_fractal_tracker',
        name='aruco_fractal_tracker',
        parameters=[{
            'marker_configuration': LaunchConfiguration('marker_configuration'),
            'marker_size': LaunchConfiguration('marker_size'),
            'min_tracking_z': 0.15,
            'max_tracking_z': 20.0,
            'max_pose_jump_m': 2.0,
            'acquire_good_frames': 5,
            'lost_bad_frames': 3,
            'show_latency_overlay': True,
            'latency_warn_ms': 100.0,
            'use_sim_time': False,
            # Camera-to-body sign mapping for real SIYI A8 Mini
            # pointing straight down: camera X = body East, camera Y = body South
            'camera_x_to_body_east_sign': -1.0,
            'camera_y_to_body_north_sign': 1.0,
            # Camera offset from drone center (set to 0 when testing on bench)
            'camera_offset_x': 0.0,
            'camera_offset_y': 0.0,
        }],
        remappings=[
            ('image_input_topic', '/siyi/image_raw'),
            ('camera_info_topic', '/siyi/camera_info'),
            ('image_output_topic', '/siyi/fractal_debug'),
            ('poses_output_topic', '/siyi/fractal_pose'),
            ('target_output_topic', '/siyi/landing_target'),
        ],
        output='screen'
    )

    return LaunchDescription([
        rtsp_url_arg,
        flip_180_arg,
        marker_configuration_arg,
        marker_size_arg,
        enable_mavros_arg,
        fcu_url_arg,
        mavros_launch,
        rtsp_node,
        tracker_node,
    ])
