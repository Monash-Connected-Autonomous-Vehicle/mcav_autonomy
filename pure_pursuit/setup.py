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
            'ppsimple_node = pure_pursuit.ppsimple_node:main',
            'ppcarla_node = pure_pursuit.ppcarla_node:main',
            'carlaSpawnerWaypoint_node = pure_pursuit.carlaSpawnerWaypoint_node:main',
        ],
    },
)
