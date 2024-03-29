'''
Launch package nodes.
- Simulation
- Robot Model
- Robot control (IK)
- Camera publisher
- Object (Cirlce) detection (Camera subscriber)
- Triangulation
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
    package_name     = 'main_package'
    dnf_package_name = 'dnf_package'
    jc_package_name  = 'jetmax_control'
    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    pkg_share_dir    = get_package_share_directory(package_name)

    # Gazebo launch
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
    #     )
    # )

    gazebo_params_path = os.path.join(
        pkg_share_dir,'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
    )
    
    # Description: This is the launch file of the robot control - This is where the controllers are loaded, configured and started
    # This is also where the robot is spawned in gazebo
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

    # Description: This is the launch file of the DNF package (not finished)
    # node_dnf = Node(
    #     package=dnf_package_name,
    #     executable="dnf_pubsub"
    # )

    node_ik_service = Node(
        package=jc_package_name,
        executable="ik_service.py"
    )

    node_fk_service = Node(
        package=jc_package_name,
        executable="fk_service.py"
    )
    
    jetmax_tcp_listener = Node(
        package=package_name, 
        executable="transform_pubsub.py"
    )

    node_triangulation = Node(
        package=package_name,
        executable="triangulation_pubsub.py"
    )




    # Description: Environment launch file
    environment = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_share_dir, 'worlds', 'environment3.world'), ''],
        description='MY DESCRIPTION, BIG WORLD, BIG DREAMS')
    
    
    # Simple service client node that moves the robot to a desired position (Home, outside the table)
    home_command_jetmax = Node(
        package='jetmax_control',
        executable='go_home_ik_client.py',
    )

    node_datalogging_ik = Node(
        package='data_logging',
        executable='datalog_IK.py'
    )
    

    return LaunchDescription([
        environment,                # Set environment world file
        gazebo,                     # Launch gazebo
        node_ik_service,             # Launch ik service
        #node_fk_service,            # Launch fk service
        node_campubsub,             # Launch camera publisher and subscriber
        #node_circlesub,             # Launch circle subscriber (triangulation in cpp)
        # node_dnf,                 # Launch DNF package node
        robot_control_launch,       # Launch robot control launch file and spawn robot in gazebo
        node_triangulation,         # Launch triangulation node
        #jetmax_tcp_listener         # Launch jetmax tcp listener - get TCP position and orientation from gazebo
        #node_datalogging_ik,
        TimerAction( # Delayed node launch ( Strongly discouraged to use, but it works - should be replaced with states )
            period=10.0, # Delay in seconds
            actions=[home_command_jetmax]
        )
        # On exit of gazebo, launch home command jetmax
        
        
    ])
