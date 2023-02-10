from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # control_spawner = Node(package='controller_manager',
    #                         executable='spawner.py',
    #                         name='controller_spawner',
    #                         output='screen',
    #                         # parameters=[{
    #                         #     'file': "$(find jetmax_control)/config/controllers.yaml",
    #                         #     'command': "load"
    #                         # }],
    #                         arguments=["joint_state_broadcaster"#,
    #                                 #"joints_effort_controllers"
    #                                 # "joint2",
    #                                 # "joint3",
    #                                 # "joint4",
    #                                 # "joint5",
    #                                 # "joint6"
    #                                 ],
    #                         namespace='/jetmax'
    #                         )

    # controller_manager_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, diffbot_diff_drive_controller],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     },
    # )

    # Get URDF via xacro
    xacro_file = os.path.join(get_package_share_directory('jetmax_description'), 'urdf', 'jetmax.xacro')
    doc = xacro.parse(open(xacro_file)) # This is where the URDF is parsed
    xacro.process_doc(doc) # This is where the URDF is generated
    robot_description = {"robot_description": doc.toxml()}

    # Description: This node is responsible for loading the controllers specified in the controllers.yaml file
    jetmax_controllers = PathJoinSubstitution(
        [
            FindPackageShare("jetmax_control"),
            "config",
            "controllers.yaml",
        ]
    )

    # Description: This node is responsible for publishing the state of the robot to the topic "robot_description"
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # print("robot_description: ", robot_description)
    # print("jetmax_controllers: ", jetmax_controllers)

    # Description: This node is responsible for loading the controllers specified in the config file
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #name="jetmax_controller_manager_node",
        parameters=[robot_description, jetmax_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Description: This node spawns the controller named "joint_state_broadcaster"
    control_spawner_jsb = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        namespace='jetmax',
        output="screen",
    )

    # Description: This node spawns the controller named "joints_effort_controller"
    control_spawner_effort = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joints_effort_controller"],
        #namespace='/jetmax',
        output="screen",
    )


    return LaunchDescription([
        node_robot_state_publisher, # Publish urdf to topic 'robot_description'
        controller_manager_node,    # Start the controller manager, using config and urdf
        control_spawner_jsb,        # Spawn the joint_state_broadcaster controller
        #control_spawner_effort,     # Spawn the joints_effort_controller controller
    ])
