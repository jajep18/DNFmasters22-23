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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import LocalSubstitution
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

    jetmax_include = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    "src/main_package/jetmax_gazebo/jetmax_control/launch/jetmax_control.launch",             
                )
            ),
            launch_arguments={
                "use_sim_time": True,
                "name": "TestName"
            }.items()



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






    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'verbose',
        #     default_value='true',
        #     description='Extra runtime info.'),
        DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_dolly_gazebo, 'worlds', 'jetmax_empty.world'), ''],
        description='MY DESCRIPTION, BIG WORLD, BIG DREAMS'),
        jetmax_include,
        gazebo
        #node_listener,
        #node_talker,
        #node_campubsub,   
        #node_circlesub,
        # spawn_entity,
        #node_dnf,
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='jetmax',
        #     # description='Publishes the state of the JetMax robot',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        #     # arguments=['src/main_package/jetmax_gazebo/jetmax_description/launch/jetmax_description.xml'] #src/main_package/jetmax_gazebo/jetmax_description/launch$ 
        #     arguments=['src/main_package/models/jetmax_description/urdf/jetmax_model.urdf'] 
        # )
    ])
