from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='apriltag_landing',
        description='Name of the Gazebo simulation world'
    )

    cruise_alt_arg = DeclareLaunchArgument(
        'cruise_alt',
        default_value='10.0',
        description='Cruise altitude during the search phase'
    )

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

    target_tag_id_arg = DeclareLaunchArgument(
        'target_tag_id',
        default_value='0',
        description='Target AprilTag ID'
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

    # 2. Gazebo Image Bridge Node (Dynamic world name using PythonExpression)
    gz_image_topic = PythonExpression([
        "'/' + 'world/' + '",
        LaunchConfiguration('world'),
        "' + '/model/x500_gimbal_0/link/camera_link/sensor/camera/image'"
    ])

    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[gz_image_topic],
        remappings=[
            (gz_image_topic, '/gimbal_camera')
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

    # Gazebo Camera Info Bridge Node (Dynamic world name using PythonExpression)
    gz_info_topic = PythonExpression([
        "'/' + 'world/' + '",
        LaunchConfiguration('world'),
        "' + '/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info'"
    ])

    gz_info_bridge_arg = PythonExpression([
        "'/' + 'world/' + '",
        LaunchConfiguration('world'),
        "' + '/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'"
    ])

    info_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[gz_info_bridge_arg],
        remappings=[
            (gz_info_topic, '/gimbal_camera/camera_info')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 4. AprilTag Tracker Node (passes coordinate transform configs)
    tracker_node = Node(
        package='aruco_fractal_tracker',
        executable='apriltag_tracker',
        parameters=[{
            'dictionary': 'DICT_APRILTAG_25h9',
            'target_tag_id': LaunchConfiguration('target_tag_id'),
            'marker_size': 0.50,
            'pose_output_topic': '/apriltag_tracker/pose',
            'use_sim_time': True,
            'camera_x_to_body_east_sign': 1.0,
            'camera_y_to_body_north_sign': -1.0,
            'camera_offset_x': LaunchConfiguration('camera_offset_x'),
            'camera_offset_y': LaunchConfiguration('camera_offset_y'),
        }],
        output='screen'
    )

    # 5. MAVROS Lander Node
    lander_node = Node(
        package='px4_offboard',
        executable='apriltag_precision_lander',
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
            'pose_topic': '/apriltag_tracker/pose'
        }],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        cruise_alt_arg,
        camera_offset_x_arg,
        camera_offset_y_arg,
        target_tag_id_arg,
        mavros_launch,
        image_bridge_node,
        clock_bridge_node,
        info_bridge_node,
        tracker_node,
        lander_node
    ])
