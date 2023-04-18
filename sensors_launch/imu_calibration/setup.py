from setuptools import setup

package_name = 'imu_calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lbwise',
    maintainer_email='wiseliam7@gmail.com',
    description='Node that publishes a CAN Frame to /to_can topic, for calibrating the IMU to its stationary values',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration = imu_calibration.calibration:main'
        ],
    },
)
