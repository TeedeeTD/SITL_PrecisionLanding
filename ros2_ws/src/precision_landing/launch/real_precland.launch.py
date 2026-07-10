"""
Launch file: Real SIYI A8 Mini + Fractal ArUco Tracker + Offboard Precision Landing Controller + MAVROS (All C++)

Pipeline:
  1. MAVROS — connects to FCU (optional, can skip if no FCU connected)
  2. siyi_camera_bridge — RTSP stream → /siyi/image_raw + /siyi/camera_info (C++ rtsp_publisher)
  3. aruco_fractal_tracker — detect fractal marker, publish pose + debug image (C++ tracker)
  4. offboard_precland_controller — controls UAV for precision landing using tracker pose (C++ controller)
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
    pkg_share = get_package_share_directory('precision_landing')
    rtsp_params_file = os.path.join(pkg_share, 'config', 'rtsp_publisher_params.yaml')
    offboard_params_file = os.path.join(pkg_share, 'config', 'offboard_precland_params.yaml')

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
            'PX4/examples/SITL_PrecisionLanding/px4/Tools/simulation/gz/models/'
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

    camera_fx_arg = DeclareLaunchArgument(
        'camera_fx',
        default_value='735.788640',
        description='Calibrated camera focal length fx'
    )

    camera_fy_arg = DeclareLaunchArgument(
        'camera_fy',
        default_value='733.752679',
        description='Calibrated camera focal length fy'
    )

    camera_cx_arg = DeclareLaunchArgument(
        'camera_cx',
        default_value='657.226337',
        description='Calibrated camera principal point cx'
    )

    camera_cy_arg = DeclareLaunchArgument(
        'camera_cy',
        default_value='350.211265',
        description='Calibrated camera principal point cy'
    )

    camera_offset_x_arg = DeclareLaunchArgument(
        'camera_offset_x',
        default_value='0.0',
        description='Camera physical X offset relative to drone center (set 0.0 for bench test)'
    )

    camera_offset_y_arg = DeclareLaunchArgument(
        'camera_offset_y',
        default_value='0.0',
        description='Camera physical Y offset relative to drone center (set 0.0 for bench test)'
    )

    camera_x_to_body_east_sign_arg = DeclareLaunchArgument(
        'camera_x_to_body_east_sign',
        default_value='-1.0',
        description='Camera X to body East mapping sign for real camera straight down'
    )

    camera_y_to_body_north_sign_arg = DeclareLaunchArgument(
        'camera_y_to_body_north_sign',
        default_value='1.0',
        description='Camera Y to body North mapping sign for real camera straight down'
    )

    # ── 1. MAVROS (optional) ────────────────────────────────────────

    mavros_launch = OpaqueFunction(function=_maybe_start_mavros)

    # ── 2. SIYI RTSP Camera Bridge ─────────────────────────────────

    rtsp_node = Node(
        package='precision_landing',
        executable='rtsp_publisher',
        name='siyi_rtsp_publisher',
        parameters=[
            rtsp_params_file,
            {
                'rtsp_url': LaunchConfiguration('rtsp_url'),
                'flip_180': LaunchConfiguration('flip_180'),
                'camera_fx': LaunchConfiguration('camera_fx'),
                'camera_fy': LaunchConfiguration('camera_fy'),
                'camera_cx': LaunchConfiguration('camera_cx'),
                'camera_cy': LaunchConfiguration('camera_cy'),
            }
        ],
        output='screen'
    )

    # ── 3. Fractal ArUco Tracker ────────────────────────────────────

    tracker_node = Node(
        package='precision_landing',
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
            'camera_x_to_body_east_sign': LaunchConfiguration('camera_x_to_body_east_sign'),
            'camera_y_to_body_north_sign': LaunchConfiguration('camera_y_to_body_north_sign'),
            'camera_offset_x': LaunchConfiguration('camera_offset_x'),
            'camera_offset_y': LaunchConfiguration('camera_offset_y'),
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

    # ── 4. Offboard Precision Landing Controller ────────────────────

    controller_node = Node(
        package='precision_landing',
        executable='offboard_precland_controller',
        name='offboard_precland_controller',
        parameters=[
            offboard_params_file,
            {
                'use_sim_time': False,
                'target_topic': '/siyi/landing_target',
                'target_pose_topic': '/siyi/fractal_pose',
                'align_yaw_to_tag': True,
            }
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
        camera_fx_arg,
        camera_fy_arg,
        camera_cx_arg,
        camera_cy_arg,
        camera_offset_x_arg,
        camera_offset_y_arg,
        camera_x_to_body_east_sign_arg,
        camera_y_to_body_north_sign_arg,
        mavros_launch,
        rtsp_node,
        tracker_node,
        controller_node,
    ])
