import os
from glob import glob
from setuptools import setup

package_name = 'obstacle_tracker'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zyad',
    maintainer_email='zyadalshujaa@gmail.com',
    description='Track moving obstacles from a 2D laser scan (Kalman filter)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tracker = obstacle_tracker.tracker_node:main',
            'tracker_eval = obstacle_tracker.tracker_eval:main',
        ],
    },
)
