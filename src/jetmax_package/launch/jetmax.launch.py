'''
Launch package nodes.
'''

from launch import LaunchDescription
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name     = 'jetmax_package'

    node_listener = Node(
        package=package_name,
        executable='my_listener'
    )

    node_talker = Node(
        package=package_name,
        executable='my_talker'
    )


    return LaunchDescription([
        node_listener,
        node_talker 
    ])