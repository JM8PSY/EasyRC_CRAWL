from glob import glob

from setuptools import find_packages, setup

package_name = 'cugo_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jm8psy',
    maintainer_email='jm8psy@users.noreply.github.com',
    description='CuGo V3 ESP32 base bridge (cmd_vel <-> USB serial)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cugo_base_node = cugo_base.node:main',
            'cugo_calibrate = cugo_base.calibrate:main',
        ],
    },
)
