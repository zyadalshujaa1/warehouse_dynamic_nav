import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('obstacle_tracker'), 'config', 'tracker.yaml')
    return LaunchDescription([
        Node(package='obstacle_tracker', executable='tracker', name='obstacle_tracker',
             parameters=[cfg, {'use_sim_time': True}], output='screen'),
    ])
