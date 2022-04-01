import os
from setuptools import setup
import glob

package_name = 'project_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name +'/data/velodyne_points/', glob.glob('data/velodyne_points/*')),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benca',
    maintainer_email='bedw0004@student.monash.edu',
    description='Package to take LiDAR sensor data and inputs from Multi-Task Panoptic Perception model to create accurate pose and velocity estimates of an object over time in 3D space.',
    license='The MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_pub = project_tracker.mock_pub:main',
            'cluster = project_tracker.cluster:main',
        ],
    },
)
