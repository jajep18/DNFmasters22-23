from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit


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
    
    # Description: This cmd loads the controller named "joint_state_broadcaster"
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen',
    )

    # Description: This cmd loads the controller named "joints_effort_controller"
    load_joint_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
                'joint_effort_controller'],
        output='screen',
    )
    # Description: This cmd loads the controller named "joints_position_controller"
    load_joint_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_position_controller'],
        output='screen',
    )

    # Description: This is were all the nodes are used to create the launch file
    return LaunchDescription([
        # Load the joint_state_broadcaster when the robot is spawned in Gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_robot,
                on_exit=[load_joint_state_broadcaster],
            ),
        ),
        # Load joint_position_controller when joint_state_broadcaster is loaded
        RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_position_controller],
            ),
        ),
        # # Load joint_effort_controller when joint_position_controller is loaded
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_position_controller,
        #         on_exit=[load_joint_effort_controller],
        #     ),
        # ),
        robot_state_publisher,      # Publish urdf to topic 'robot_description'
        spawn_entity_robot,         # Spawn robot in Gazebo
    ])
