from glob import glob
import os
from setuptools import setup

package_name = 'supervisor'

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
    maintainer='owenbrooks',
    maintainer_email='owen.h.brooks@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor = supervisor.supervisor:main',
            'minimal_publisher = supervisor.minimal_publisher:main',
        ],
    },
)
