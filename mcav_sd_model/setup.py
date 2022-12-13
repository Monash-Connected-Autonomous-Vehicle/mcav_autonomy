from setuptools import setup
from glob import glob

package_name = 'mcav_sd_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.*')),
        # had to add lots of entries since it doesn't like copying directories recursively
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf*')),
        ('share/' + package_name + '/meshes', glob('meshes/*.*')),
        ('share/' + package_name + '/urdf/bases', glob('urdf/bases/*.urdf*')),
        ('share/' + package_name + '/urdf/bases', glob('urdf/bases/*.xacro*')),
        ('share/' + package_name + '/urdf/sensors', glob('urdf/sensors/*.urdf*')),
        ('share/' + package_name + '/urdf/wheels', glob('urdf/wheels/*.urdf*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcav',
    maintainer_email='jaisinghanilaksh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
