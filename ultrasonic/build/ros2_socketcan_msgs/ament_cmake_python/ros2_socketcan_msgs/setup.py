from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_socketcan_msgs',
    version='1.2.0',
    packages=find_packages(
        include=('ros2_socketcan_msgs', 'ros2_socketcan_msgs.*')),
)
