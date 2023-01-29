from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
## Blob test includes
import xacro
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    x = DeclareLaunchArgument(
        'x', default_value='0',
        description='X coordinate of the robot pose'
    )
    y = DeclareLaunchArgument(
        'y', default_value='0',
        description='Y coordinate of the robot pose'
    )
    z = DeclareLaunchArgument(
        'z', default_value='0',
        description='Z coordinate of the robot pose'
    )
    roll = DeclareLaunchArgument(
        'roll', default_value='0',
        description='Roll angle of the robot pose'
    )
    pitch = DeclareLaunchArgument(
        'pitch', default_value='0',
        description='Pitch angle of the robot pose'
    )
    yaw = DeclareLaunchArgument(
        'yaw', default_value='0',
        description='Yaw angle of the robot pose'
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    #xacro_file = get_package_share_directory('jetmax_description') + '/urdf/jetmax.xacro'
    urdf_file = get_package_share_directory('jetmax_description') + '/urdf/jetmax.urdf'
    sdf_file = get_package_share_directory('jetmax_description') + '/urdf/jetmax.sdf'
    urdf = os.path.join(get_package_share_directory('jetmax_description'), 'urdf', 'jetmax.urdf')

    xacro_file = os.path.join(get_package_share_directory('jetmax_description'), 'urdf', 'jetmax.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}


	# Robot State Publisher 
    robot_state_publisher = Node(package    ='robot_state_publisher',
								 executable ='robot_state_publisher',
								 name       ='robot_state_publisher',
								 output     ='both',
								#  parameters =[{'robot_description': Command(['xacro', ' ', xacro_file])           
                                #  parameters =[{'robot_description': urdf}]
                                #  arguments=[urdf]
                                 parameters=[params]
                                #  arguments=['-x', LaunchConfiguration('x'),
                                #             '-y', LaunchConfiguration('y'),
                                #             '-z', LaunchConfiguration('z'),
                                #             '-R', LaunchConfiguration('roll'),
                                #             '-P', LaunchConfiguration('pitch'),
                                #             '-Y', LaunchConfiguration('yaw')]
                                )


	# Spawn the robot in Gazebo
    spawn_entity_robot = Node(package     ='gazebo_ros', 
							  executable  ='spawn_entity.py', 
							  arguments   = ['-entity', 'jetmax', '-topic', 'robot_description'],
							  output      ='screen')

    # urdf_jetmax_action = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='urdf_jetmax',
    #     output='screen',
    #     arguments=['-entity', 'jetmax',
    #                '-file', urdf_file, #'$(find jetmax_description)/urdf/jetmax.urdf',
    #                '-x', LaunchConfiguration('x'),
    #                '-y', LaunchConfiguration('y'),
    #                '-z', LaunchConfiguration('z'),
    #                '-R', LaunchConfiguration('roll'),
    #                '-P', LaunchConfiguration('pitch'),
    #                '-Y', LaunchConfiguration('yaw')],
    # )


    # spawn_jetmax = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='jetmax_spawner',
    #     namespace='jetmax',
    #     arguments=['-database', 'urdf_jetmax', '-entity', 'jetmax',
    #                '-x', '0', '-y', '0', '-z', '0', '-R', '0', '-P', '0', '-Y', '0'],
    #     output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controllers'],
        output='screen'
    )
    
    return LaunchDescription([
        # RegisterEventHandler( #When the gazebo process is finished, load the joint state controller
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity_robot,
        #         on_exit=[load_joint_state_controller],
        #     )
        # ),
        # RegisterEventHandler( #When the joint state controller is loaded, load the joint trajectory controller
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        gazebo,
        x, y, z, roll, pitch, yaw,  # Launch argumentes
        #urdf_jetmax_action
        robot_state_publisher,
        spawn_entity_robot
    ])