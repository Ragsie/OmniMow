"""
OmniMow Nav2 bringup for ROS 2 Jazzy.

Starts only the Nav2 servers OmniMow uses. nav2_bringup's navigation_launch.py also starts
route_server, docking_server, smoother_server and waypoint_follower, which each need their own
configuration and would stop the lifecycle bringup if they fail to configure.

Command path: controller/behaviors -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
-> collision_monitor -> /cmd_vel (Drive ESP32).
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
LIFECYCLE_NODES = ['controller_server', 'planner_server', 'behavior_server',
                   'velocity_smoother', 'collision_monitor', 'bt_navigator']


def generate_launch_description():
    # Fill in the robot dimensions from omnimow.env (plain replace; the YAML has no other braces)
    with open(os.path.join(HERE, 'nav2_params.yaml')) as f:
        params = f.read()
    params = params.replace('{FOOTPRINT}', os.getenv(
        'OMNIMOW_FOOTPRINT', '[[0.30, 0.20], [0.30, -0.20], [-0.25, -0.20], [-0.25, 0.20]]'))
    params = params.replace('{INFLATION_RADIUS}', os.getenv('OMNIMOW_NAV2_INFLATION_RADIUS', '0.48'))
    params_file = '/tmp/nav2_params_resolved.yaml'
    with open(params_file, 'w') as f:
        f.write(params)
    print(f"[OmniMow] Nav2 params: footprint={os.getenv('OMNIMOW_FOOTPRINT', 'default')} "
          f"inflation={os.getenv('OMNIMOW_NAV2_INFLATION_RADIUS', '0.48')}")

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[params_file], remappings=list(remappings))

    return LaunchDescription([
        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_collision_monitor', 'collision_monitor'),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': LIFECYCLE_NODES, 'bond_timeout': 10.0}]),
    ])
