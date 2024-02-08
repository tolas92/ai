from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory

import os

def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    image_topic_dec = DeclareLaunchArgument(
        'image_topic',
        default_value='/color/video/image',
        description='The name of the input image topic.')
    
    publish_topic = LaunchConfiguration('publish_topic')
    publish_topic_dec = DeclareLaunchArgument(
        'publish_topic',
        default_value='/table_number',
        description='The name of the input image topic.')
    
    #launching the hand_gesture_node
    hand_control_node=Node(
            package='hand_control',
            executable='hand_control',
            remappings=[('/image_raw',image_topic),
                        ('/hand_gesture',publish_topic)],
            name='hand_sign')
    #node to open the camera
    camera_launch_node=Node(
        package='depthai_examples',
        executable='rgb_stereo_node',
        parameters=[{'useDepth':False}],
    )
    return LaunchDescription([
        image_topic_dec,
        publish_topic_dec,
        hand_control_node,
        camera_launch_node
  ])