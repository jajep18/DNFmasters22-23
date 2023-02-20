'''
Launch package nodes.
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

def generate_launch_description():

    package_name     = 'main_package'
    dnf_package_name = 'dnf_package'
    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    pkg_share_dir    = get_package_share_directory(package_name)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    
    # Description: This is the launch file of the robot control - This is where the controllers are loaded, configured and started
    # Launch another launch file inside this one
    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('jetmax_control'), 'launch', 'inc'), '/robot_control.launch.py']),
    ) 

    # Description: This is the launch file of the camera publisher and subscriber
    node_campubsub = Node(
        package=package_name,
        executable='cam_pubsub'
    )

    # Description: This is the launch file of the circle subscriber, which subscribes to the camera topic
    node_circlesub = Node(
        package=package_name,
        executable='circle_listener'
    )      

    node_dnf = Node(
        package=dnf_package_name,
        executable="dnf_pubsub"
    )


    return LaunchDescription([
        DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_share_dir, 'worlds', 'environment3.world'), ''],
        description='MY DESCRIPTION, BIG WORLD, BIG DREAMS'),
        gazebo,
        node_campubsub,   
        node_circlesub,
        # node_dnf,
        robot_control_launch    
    ])
