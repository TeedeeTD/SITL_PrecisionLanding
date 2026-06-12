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
            'fractal_aruco_precision_lander = px4_offboard.aruco_precision_lander:fractal_main',
            'apriltag_precision_lander = px4_offboard.apriltag_precision_lander:main',
            'fiducial_detector_node = px4_offboard.fiducial_detector_node:main',
        ],
    },
)
