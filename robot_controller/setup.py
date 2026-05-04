from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament_index registration
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # config files (nav2_params.yaml 등)
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Autonomous skid-steer robot controller (ROS 2 Jazzy) + Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node          = robot_controller.lidar_node:main',
            'motor_node          = robot_controller.motor_node:main',
            'avoidance_node      = robot_controller.avoidance_node:main',
            'keyboard_node       = robot_controller.keyboard_node:main',
            'nav2_goal_publisher = robot_controller.nav2_goal_publisher:main',
        ],
    },
)
