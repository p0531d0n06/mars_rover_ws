from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mars_rover_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mars_rover',
    maintainer_email='rover@mars.local',
    description='ArUco marker detection and Nav2 waypoint bridge for the Mars rover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_waypoint_bridge = mars_rover_aruco.aruco_waypoint_bridge:main',
            'aruco_detector = mars_rover_aruco.aruco_detector:main',
        ],
    },
)
