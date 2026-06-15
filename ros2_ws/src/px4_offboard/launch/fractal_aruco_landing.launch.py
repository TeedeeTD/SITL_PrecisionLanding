from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    marker_configuration_arg = DeclareLaunchArgument(
        'marker_configuration',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'PX4/Tools/simulation/gz/models/fractal_aruco_marker/custom_fractal.yml'
        ),
        description='Absolute path to the custom fractal marker configuration'
    )

    # Declare cruise_alt launch argument
    cruise_alt_arg = DeclareLaunchArgument(
        'cruise_alt',
        default_value='10.0',
        description='Cruise altitude during the search phase'
    )

    # Declare camera physical offsets in body FLU frame
    camera_offset_x_arg = DeclareLaunchArgument(
        'camera_offset_x',
        default_value='0.1517',
        description='Camera physical X offset relative to drone center in body FLU frame'
    )
    camera_offset_y_arg = DeclareLaunchArgument(
        'camera_offset_y',
        default_value='0.0',
        description='Camera physical Y offset relative to drone center in body FLU frame'
    )

    # 1. Include MAVROS px4.launch
    mavros_dir = get_package_share_directory('mavros')
    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(mavros_dir, 'launch', 'px4.launch')
        ),
        launch_arguments={
            'fcu_url': 'udp://:14540@127.0.0.1:14580',
        }.items()
    )

    # 2. Gazebo Image Bridge Node
    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image'],
        remappings=[
            ('/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image', '/gimbal_camera')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 3. Gazebo Clock Bridge Node
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 4. C++ Tracker Node
    tracker_node = Node(
        package='aruco_fractal_tracker',
        executable='aruco_fractal_tracker',
        parameters=[{
            'marker_configuration': LaunchConfiguration('marker_configuration'),
            'marker_size': 0.50,
            'show_latency_overlay': True,
            'latency_warn_ms': 100.0,
            'use_sim_time': True,
            'camera_x_to_body_east_sign': 1.0,
            'camera_y_to_body_north_sign': -1.0,
            'camera_offset_x': LaunchConfiguration('camera_offset_x'),
            'camera_offset_y': LaunchConfiguration('camera_offset_y')
        }],
        remappings=[
            ('image_input_topic', '/gimbal_camera'),
            ('camera_info_topic', '/gimbal_camera/camera_info'),
            ('image_output_topic', '/landing/annotated_image'),
            ('poses_output_topic', '/aruco_fractal_tracker/poses')
        ],
        output='screen'
    )

    # 5. MAVROS Lander Node
    lander_node = Node(
        package='px4_offboard',
        executable='fractal_aruco_precision_lander',
        parameters=[{
            'search_frame': 'enu',
            'search_x': 3.0,
            'search_y': 2.0,
            'cruise_alt': LaunchConfiguration('cruise_alt'),
            'camera_yaw_frame': 'body',
            'camera_x_to_body_east_sign': 1.0,
            'camera_y_to_body_north_sign': -1.0,
            'camera_offset_x': LaunchConfiguration('camera_offset_x'),
            'camera_offset_y': LaunchConfiguration('camera_offset_y'),
            'use_sim_time': True,
            'pose_topic': '/aruco_fractal_tracker/poses'
        }],
        output='screen'
    )

    return LaunchDescription([
        marker_configuration_arg,
        cruise_alt_arg,
        camera_offset_x_arg,
        camera_offset_y_arg,
        mavros_launch,
        image_bridge_node,
        clock_bridge_node,
        tracker_node,
        lander_node
    ])
