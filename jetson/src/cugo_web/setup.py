from glob import glob

from setuptools import find_packages, setup

package_name = 'cugo_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jm8psy',
    maintainer_email='jm8psy@users.noreply.github.com',
    description='CuGo web UI (teleop, map view, Nav2 goals) served from the Jetson',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cugo_web_node = cugo_web.node:main',
        ],
    },
)
