from setuptools import setup
import os

package_name = 'drone_ml'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salman',
    maintainer_email='drone@example.com',
    description='RL Drone Node for PX4',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_node = drone_ml.drone_rl_node:main',
            'teleop = drone_ml.teleop_keyboard:main',
            'follow_cam = drone_ml.follow_cam:main',
            'qgc_stream = drone_ml.stream_video:main',
        ],
    },
)
