# from setuptools import setup

# package_name = 'mujoco_multi_agv'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     entry_points={
#         'console_scripts': [
#             'mujoco_sim_node = mujoco_multi_agv.mujoco_sim_node:main'
#         ],
#     },
# )

# from setuptools import setup
# import os
# from glob import glob

# package_name = 'mujoco_multi_agv'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         (os.path.join('share', package_name, 'launch'),
#             glob('launch/*.py')),
#         (os.path.join('share', package_name, 'scenes'),
#             glob('scenes/*.xml')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     entry_points={
#         'console_scripts': [
#             'mujoco_sim_node = mujoco_multi_agv.mujoco_sim_node:main'
#         ],
#     },
# )

# from setuptools import setup
# import os
# from glob import glob

# package_name = 'mujoco_multi_agv'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         (os.path.join('share', package_name, 'launch'),
#             glob('launch/*.py')),
#         (os.path.join('share', package_name, 'scenes'),
#             glob('scenes/*.xml')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='andy',
#     maintainer_email='andy@example.com',
#     description='MuJoCo multi robot simulation',
#     license='Apache License 2.0',
#     entry_points={
#         'console_scripts': [
#             'mujoco_sim_node = mujoco_multi_agv.mujoco_sim_node:main'
#         ],
#     },
# )


# from setuptools import setup
# import os
# from glob import glob

# package_name = 'mujoco_multi_agv'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         (os.path.join('share', package_name, 'launch'),
#             glob('launch/*.py')),
#         (os.path.join('share', package_name, 'scenes'),
#             glob('scenes/*.xml')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='andy',
#     maintainer_email='andy@example.com',
#     description='MuJoCo multi robot simulation',
#     license='Apache License 2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'mujoco_sim_node = mujoco_multi_agv.mujoco_sim_node:main',
#         ],
#     },
# )


# from setuptools import setup
# from glob import glob
# import os

# package_name = 'mujoco_multi_agv'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/launch', glob('launch/*.py')),
#         ('share/' + package_name + '/scenes', glob('scenes/*.xml')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='andy',
#     maintainer_email='andy@example.com',
#     description='MuJoCo multi robot simulation',
#     license='Apache-2.0',
#     entry_points={
#         'console_scripts': [
#             'mujoco_sim_node = mujoco_multi_agv.mujoco_sim_node:main',
#         ],
#     },
# )

from setuptools import setup
from glob import glob

package_name = 'mujoco_multi_agv'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # folder with __init__.py
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/scenes', glob('scenes/*.xml')),
        (f'share/{package_name}/models/assets', glob('models/assets/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='andy@example.com',
    description='MuJoCo multi robot simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = mujoco_multi_agv.mujoco_sim_node:main',
        ],
    },
)