from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
## Blob test includes
import xacro
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Generate URDF file via xacro
    xacro_file = os.path.join(get_package_share_directory('jetmax_description'), 'urdf', 'jetmax.xacro')
    doc = xacro.parse(open(xacro_file)) # This is where the URDF is parsed
    xacro.process_doc(doc) # This is where the URDF is generated
    robot_description = {"robot_description": doc.toxml()}

	# Description: This node is responsible for publishing the state of the robot (aka the URDF file) to the topic "robot_description"
    robot_state_publisher = Node(package="robot_state_publisher",
                                 executable="robot_state_publisher",
                                 output="screen",
                                 parameters=[robot_description])
        # This node publishes the entity from the jetmax urdf to the topic robot_description
        # This topic is used by the spawn_entity.py node to spawn the robot in Gazebo


	# Description: This node will spawn the robot in Gazebo, using the URDF published to the topic robot_description
    spawn_entity_robot = Node(package     ='gazebo_ros', 
							  executable  ='spawn_entity.py', 
							  arguments   = ['-entity', 'jetmax', '-topic', 'robot_description'],
							  output      ='screen')
        # Spawn the robot in Gazebo, loads the entity published in the topic robot_description
        # The robot should be published in the topic robot_description before this node is executed
        # This is done using the robot_state_publisher node
    
    return LaunchDescription([
        robot_state_publisher,  # Publish urdf to topic 'robot_description'
        spawn_entity_robot,     # Spawn robot in Gazebo
    ])