from setuptools import find_packages, setup

package_name = 'inverted_pendulum_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/cart_controllers.yaml', 'config/controllers.yaml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'python-control'  # For control systems library
    ],
    zip_safe=True,
    maintainer='harris',
    maintainer_email='yuopres@gmail.com',
    description='Inverted Pendulum Controller using LQR',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'inverted_pendulum_controller = inverted_pendulum_controller.inverted_pendulum_control:main',
        ],
    },
)
