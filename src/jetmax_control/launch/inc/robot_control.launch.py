from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Generate URDF file via xacro
    xacro_file = os.path.join(get_package_share_directory('jetmax_description'), 'urdf', 'jetmax.xacro')
    doc = xacro.parse(open(xacro_file)) # This is where the URDF is parsed
    xacro.process_doc(doc) # This is where the URDF is generated
    robot_description = {"robot_description": doc.toxml()}

    # Description: This variable is the path for loading the controllers.yaml file
    jetmax_controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("jetmax_control"),
            "config",
            "controllers.yaml",
        ]
    )

    # Description: This node is responsible for loading the controllers specified in the config file
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #name="/jetmax_controller_manager",
        parameters=[robot_description, jetmax_controllers_yaml],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Description: This node spawns the controller named "joint_state_broadcaster"
    control_spawner_jsb = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        namespace='jetmax',
        output="screen",
        # arguments=["joint_state_broadcaster",
        # "Joints_effort_controllers",
        # "Joint2",
        # "Joint3",
        # "Joint4",
        # "Joint5",
        # "Joint6",
        # "Joint7",
        # "Joint8",
        # "Joint9"]
    )

    # Description: This node spawns the controller named "joints_effort_controller"
    control_spawner_effort = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joints_effort_controller", "--controller-manager", "/controller_manager"],
        #namespace='/jetmax',
        output="screen",
    )

    # Description: This node spawns the controller named "joints_position_controller"
    control_spawner_position = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joints_position_controller", "--controller-manager", "/controller_manager"],
        #namespace='/jetmax',
        output="screen",
    )

    # Description: This is were all the nodes are used to create the launch file
    return LaunchDescription([
        #robot_state_publisher_node, # Publish urdf to topic 'robot_description'
        controller_manager_node,    # Start the controller manager, using config and urdf
        control_spawner_jsb,        # Spawn the joint_state_broadcaster controller
        #control_spawner_effort,     # Spawn the joints_effort_controller controller
        control_spawner_position,   # Spawn the joints_position_controller controller
    ])
