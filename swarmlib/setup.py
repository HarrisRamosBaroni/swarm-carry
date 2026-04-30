from setuptools import setup, find_packages

# find_packages() run from inside swarmlib/ discovers the sub-packages
# (controllers, communication, simulation) but not 'swarmlib' itself.
# package_dir tells setuptools that the 'swarmlib' package root is this directory.
_sub = find_packages()  # ['controllers', 'communication', 'simulation']

setup(
    name='swarmlib',
    package_dir={'swarmlib': '.'},
    packages=['swarmlib'] + [f'swarmlib.{p}' for p in _sub],
    install_requires=['numpy'],
    extras_require={
        'sim': ['mujoco'],
    },
)
