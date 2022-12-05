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

    package_name     = 'main_package'
    dnf_package_name = 'dnf_package'
    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    pkg_dolly_gazebo = get_package_share_directory(package_name)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    node_campubsub = Node(
        package=package_name,
        executable='cam_pubsub'
    )

    node_circlesub = Node(
        package=package_name,
        executable='circle_listener'
    )      

    node_dnf = Node(
        package=dnf_package_name,
        executable="dnf_pubsub"
    )


    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'verbose',
        #     default_value='true',
        #     description='Extra runtime info.'),
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_dolly_gazebo, 'worlds', 'environment2.world'), ''],
          #default_value='test_package/test_package/worlds/dolly_empty.world',
          description='MY DESCRIPTION, BIG WORLD, BIG DREAMS'),
        gazebo,
        node_campubsub,   
        #node_circlesub,
        node_dnf     
    ])
