"""Spawn the moving workers of one scenario into the running simulation.

Start the simulation first (robot_bringup igin_sim.xml), then:
    ros2 launch dynamic_obstacles workers.launch.py scenario:=crossing

Scenarios are defined in config/scenarios.yaml (demo, crossing, head_on,
overtaking). Restart the simulation before launching another scenario:
worker names are reused, and Gazebo refuses to spawn a name twice.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_scenario(context):
    share = get_package_share_directory('dynamic_obstacles')
    model = os.path.join(share, 'models', 'worker', 'model.sdf')
    with open(os.path.join(share, 'config', 'scenarios.yaml')) as f:
        scenarios = yaml.safe_load(f)

    name = LaunchConfiguration('scenario').perform(context)
    if name not in scenarios:
        raise RuntimeError(
            f"Unknown scenario '{name}'. Available: {', '.join(scenarios)}")
    scenario = scenarios[name]
    workers = scenario['workers']

    actions = [LogInfo(msg=f"Scenario '{name}': {scenario['description']}")]

    for w in workers:
        actions.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-file', model, '-name', w['name'],
                       '-x', str(w['x']), '-y', str(w['y']), '-z', '0.01']))

    # ROS -> Gazebo: velocity commands for every worker ("]" = one direction).
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        name='workers_bridge',
        arguments=[f"/model/{w['name']}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"
                   for w in workers]))

    for w in workers:
        params = {k: w[k] for k in ('dir_x', 'dir_y', 'distance', 'speed', 'loop',
                                    'trigger_x', 'trigger_y', 'trigger_radius')}
        params.update({'use_sim_time': True, 'worker_name': w['name'], 'pause': 1.0})
        actions.append(Node(
            package='dynamic_obstacles', executable='worker_mover', output='screen',
            name=f"{w['name']}_mover", parameters=[params]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='demo',
                              description='demo | crossing | head_on | overtaking'),
        OpaqueFunction(function=launch_scenario),
    ])
