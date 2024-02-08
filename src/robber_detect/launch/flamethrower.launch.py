#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    fire_node = Node(
        package='robber_detect',
        executable='flamethrower.py',
        output='screen'
    )
    detect_node=Node(
        package='robber_detect',
        executable='2detect.py',
        output='screen'
    )

    ld.add_action(fire_node)
    ld.add_action(detect_node) 

    return ld