from setuptools import setup, find_packages

setup(
    name='swarmlib',
    packages=find_packages(),
    install_requires=['numpy', 'mujoco'],
)
