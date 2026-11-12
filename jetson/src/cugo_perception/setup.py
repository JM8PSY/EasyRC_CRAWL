from glob import glob

from setuptools import find_packages, setup

package_name = 'cugo_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jm8psy',
    maintainer_email='jm8psy@users.noreply.github.com',
    description='CuGo perception: downward stair / drop-off (cliff) detection from D455',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cliff_detector = cugo_perception.cliff_node:main',
            'scan_inf = cugo_perception.scan_inf:main',
        ],
    },
)
