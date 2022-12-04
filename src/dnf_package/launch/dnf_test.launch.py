'''
Launch dnf nodes to test package.
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

    package_name     = 'dnf_package'

    node_dnf = Node(
        package=package_name,
        executable="dnf_pubsub"
    )


    return LaunchDescription([
        node_dnf     
    ])
