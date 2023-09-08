from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ultrasonic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pham Nhat Quang Nguyen',
    maintainer_email='nguyenjoe0201@gmail.com',
    description='Tutorial package run with Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reader = ultrasonic.ultrasonic:main'
        ],
    },
)
