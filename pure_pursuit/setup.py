import os
from glob import glob
from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sqiu101',
    maintainer_email='senman_q@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'purepursuit                 = pure_pursuit.purepursuit:main',
            'carla_spawner_local_planner = pure_pursuit.carla_spawner_local_planner:main',
            'carla_spawner               = pure_pursuit.carla_spawner:main',
            'carla_global_planner        = pure_pursuit.carla_global_planner:main',
        ],
    },
)
