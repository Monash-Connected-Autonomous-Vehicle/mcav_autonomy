from setuptools import setup
import os
from glob import glob

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owen',
    maintainer_email='owen.h.brooks@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_spawn = simulation.carla_spawn:main',
            'generate_traffic = simulation.generate_traffic:main',
            'simple_sim = simulation.simple_sim:main',
            'twist_stamp_remover = simulation.twist_stamp_remover:main',
        ],
    },
)
