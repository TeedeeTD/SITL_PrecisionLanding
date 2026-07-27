from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Bridge ảnh từ Gazebo sang ROS 2 chạy trên PC
    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_image_bridge',
        arguments=['/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image'],
        remappings=[
            ('/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image', '/gimbal_camera_local')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Node nén ảnh (raw -> compressed) cục bộ trên PC trước khi gửi qua mạng Wifi sang Jetson
    compress_node = Node(
        package='image_transport',
        executable='republish',
        name='image_compressor',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/gimbal_camera_local'),
            ('out/compressed', '/gimbal_camera/compressed')
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    # Bridge đồng hồ clock từ Gazebo sang ROS 2 chạy trên PC
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Bridge camera_info từ Gazebo sang ROS 2 chạy trên PC
    camera_info_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_info_bridge',
        arguments=[
            '/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info', '/gimbal_camera/camera_info')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        image_bridge_node,
        compress_node,
        clock_bridge_node,
        camera_info_bridge_node
    ])

