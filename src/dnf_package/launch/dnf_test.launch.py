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
    main_package_name= 'main_package'
    pkg_dolly_gazebo = get_package_share_directory(main_package_name)
    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    node_dnf = Node(
        package=package_name,
        executable="dnf_pubsub"
    )


    return LaunchDescription([
        # DeclareLaunchArgument(
        # 'world',
        # default_value=[os.path.join(pkg_dolly_gazebo, 'worlds', 'environment2.world'), ''],
        # description='MY DESCRIPTION, BIG WORLD, BIG DREAMS'),
        # gazebo,
        node_dnf     
    ])
