'''
Launch command center nodes.
- Decisionmaking (DNF) node
- Movement Command Center
- Audio capture node
- Audio recognition node (pocketsphinx)
'''

from launch import LaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import LocalSubstitution
from launch.substitutions import ThisLaunchFileDir, EnvironmentVariable

#Delayed node launch
from launch.actions import TimerAction

def generate_launch_description():
    # Description: Define the package names and the dnf package name for later use
    package_name        = 'main_package'
    dnf_package_name    = 'dnf_package'
    jc_package_name     = 'jetmax_control'
    audio_package_name  = 'audio_package'
    
    # Launch for the movement command center python file
    # This node is responsible for sending movement commands to the jetmax_control package
    movement_command_center_node = Node(
        package=package_name,
        executable='movement_command_center.py'
    )    

    # Description: Audio capture node
    # Records audio and saves it as a file. File name is published to a topic
    audio_capture = Node(
        package=audio_package_name,
        executable='mic_sensor'
    )

    # Description: Audio recognition node
    # Recognizes audio and publishes the keywords in the recognized text to a topic
    audio_recognition = Node(
        package=audio_package_name,
        executable='pocketsphinx_pubsub'
    )

    # Description: Decision making node (DNF)
    # This node is responsible for making decisions based on the audio keywords
    # It subscribes to the audio recognition topic and publishes movement commands to the movement command center
    decision_making = Node(
        package=dnf_package_name,
        executable='dnf_pubsub'
    )
    
    
    # Description: Return the launch description
    return LaunchDescription([
        #movement_command_center_node,
        audio_recognition,
        decision_making,
        #TimerAction(
        #    period=10.0,  # Delay in seconds
        #    actions=[audio_capture]
        #)
    ])
    