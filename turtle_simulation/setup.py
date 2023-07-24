from setuptools import setup
from glob import glob
import os

package_name = 'turtle_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcav',
    maintainer_email='surya.kannan@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stamper = turtle_simulation.pose_stamper:main',
        ],
    },
)
