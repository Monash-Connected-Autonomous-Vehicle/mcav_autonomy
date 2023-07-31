from setuptools import setup
from glob import glob
import os

package_name = 'gazebo_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch*')),
        ('share/' + package_name, ['config/gazebo_topics.yaml', 'config/lidar_playground.sdf','launch/gazebo_playground.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newuser',
    maintainer_email='anthony.zh.oon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_converter = gazebo_sim.sim_converter:main'
        ],
    },
)
