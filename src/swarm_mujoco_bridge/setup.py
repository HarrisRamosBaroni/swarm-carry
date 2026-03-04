from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'swarm_mujoco_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarm-carry team',
    maintainer_email='todo@todo.com',
    description='MuJoCo-ROS2 bridge for swarm payload transport simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = swarm_mujoco_bridge.bridge_node:main',
        ],
    },
)
