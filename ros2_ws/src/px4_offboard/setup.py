import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ducanh',
    description='PX4 Offboard Control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drone_controller = px4_offboard.drone_controller:main',
            'camera_viewer    = px4_offboard.camera_viewer:main',
            'fractal_aruco_precision_lander = px4_offboard.fractal_aruco_precision_lander:main',
            'landing_target_bridge = px4_offboard.landing_target_bridge:main',
            'offboard_precland_controller = px4_offboard.offboard_precland_controller:main',
            'apriltag_precision_lander = px4_offboard.apriltag_precision_lander:main',
            'aruco_precision_lander = px4_offboard.aruco_precision_lander:main',
            'box_hybrid_precision_lander = px4_offboard.box_hybrid_precision_lander:main',
            'box_hybrid_status_monitor = px4_offboard.box_hybrid_status_monitor:main',
            'sim_box_manager = px4_offboard.sim_box_manager:main',
            'mock_box_hardware = px4_offboard.mock_box_hardware:main',
            'mavros_to_dib_telemetry = px4_offboard.mavros_to_dib_telemetry:main',
        ],
    },
)
