import os
from glob import glob
from setuptools import setup

package_name = 'dynamic_obstacles'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models', 'worker'), glob('models/worker/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zyad',
    maintainer_email='zyadalshujaa@gmail.com',
    description='Moving worker obstacles for the warehouse simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'worker_mover = dynamic_obstacles.worker_mover:main',
            'trial_runner = dynamic_obstacles.trial_runner:main',
        ],
    },
)
