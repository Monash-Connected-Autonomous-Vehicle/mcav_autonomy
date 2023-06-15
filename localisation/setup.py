from setuptools import setup

package_name = 'localisation'

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
    maintainer='newuser',
    maintainer_email='anthony.zh.oon@gmail.com',
    description='Localisation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initialise_pose        = localisation.pose_initialisation:main',
        ],
    },
)
