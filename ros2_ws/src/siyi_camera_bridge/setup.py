import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'siyi_camera_bridge'

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
    description='RTSP camera bridge for SIYI A8 Mini to ROS2 Image topics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rtsp_publisher = siyi_camera_bridge.rtsp_publisher:main',
        ],
    },
)
