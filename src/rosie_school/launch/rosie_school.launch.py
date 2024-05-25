#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    voice_server_node = Node(
        package='rosie_school',
        executable='ai_voice_server.py',
        output='screen'
    )
    eye_animation_node=Node(
        package='rosie_school',
        executable='eyes_animation.py',
        output='screen'
    )

    ld.add_action(voice_server_node)
    ld.add_action(eye_animation_node) 

    return ld