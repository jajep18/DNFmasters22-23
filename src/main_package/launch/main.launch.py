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
    
    ### # De metoder jeg har arbejdet på: ######
    # jetmax_include = IncludeLaunchDescription( 
    #         # XMLLaunchDescriptionSource(
    #         #FrontendLaunchDescriptionSource(
    #                 "src/main_package/jetmax_gazebo/jetmax_control/launch/jetmax_control.launch",             
    #         #)
    #     )

    # def jetMaxInclude():
    #     jetmax_include = IncludeLaunchDescription(
    #         FrontendLaunchDescriptionSource(['/home/jacob/DNFmasters22-23/src/main_package/jetmax_gazebo/jetmax_control/launch/inc/jetmax_spawner.xml']),
            
    #     )
    #     return LaunchDescription([ jetmax_include ])

        # include the launch file for the robot
    # robot_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={
    #         'gazebo_models_path': 'jetmax_gazebo/jetmax_description/urdf/jetmax.xacro',
    #         'robot_name_in_model': 'jetmax',
    #         'rviz_config_file_path': 'jetmax_gazebo/jetmax_description/config/jetmax.rviz',
    #         'urdf_file_path': "jetmax_gazebo/jetmax_description/urdf",
    #         'spawn_x_val': 1.0,
    #         'spawn_y_val': 1.0,
    #         'spawn_z_val': 1.0,
    #         'spawn_yaw_val': 0.0
    #     }.items()
    # )

        ### End of metoder ###

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py', # Gazebo script der kan spawne entity
        name='jetmax_0',
        namespace='jetmax_0',
        arguments=['-database', 'jetmax', '-entity', 'jetmax_0',
                   '-x', '0', '-y', '0', '-z', '0', '-R', '0', '-P', '0', '-Y', '0'],
        output='screen'
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
        default_value=[os.path.join(pkg_share_dir, 'worlds', 'jetmax_empty.world'), ''],
        description='MY DESCRIPTION, BIG WORLD, BIG DREAMS'),
        # jetmax_include,
        gazebo,
        spawn_robot,
        #node_listener,
        #node_talker,
        #node_campubsub,   
        #node_circlesub,
        # spawn_entity,
        #node_dnf,
    ])
