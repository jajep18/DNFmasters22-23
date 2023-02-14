from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Description: This is where Gazebo is launched
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    # Description: This is the launch file of the robot control - This is where the controllers are loaded, configured and started
    # Launch another launch file inside this one
    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('jetmax_control'), 'launch', 'inc'), '/robot_control.launch.py']),
    ) 

    # Description: This is the launch file of the robot spawner - This is where the robot is spawned in Gazebo
    robot_spawner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('jetmax_control'), 'launch', 'inc'), '/robot_spawner.launch.py']),
    ) 

    return LaunchDescription([
        gazebo,
        robot_spawner_launch,
        robot_control_launch      
    ])