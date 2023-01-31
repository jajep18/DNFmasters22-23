from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    control_spawner = Node(package='controller_manager',
                            executable='spawner.py',
                            name='controller_spawner',
                            output='screen',
                            parameters=[{
                                'file': "$(find jetmax_control)/config/controllers.yaml",
                                'command': "load"
                            }],
                            arguments=["joint_state_controller",
                                    # "joint1",
                                    # "joint2",
                                    # "joint3",
                                    # "joint4",
                                    # "joint5",
                                    # "joint6"
                                    ],
                            # arguments=["joint_state_controller",
                            #         "joint1_position_controller",
                            #         "joint2_position_controller",
                            #         "joint3_position_controller",
                            #         "joint4_position_controller",
                            #         "joint5_position_controller",
                            #         "joint6_position_controller",
                            #         "joint7_position_controller",
                            #         "joint8_position_controller",
                            #         "joint9_position_controller"],
                            namespace='/jetmax'
                            )

    # delayed_controller_manager_spawner = TimerAction(
    #     period=10.0,
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner.py",
    #             arguments=["diff_cont"],
    #         ),
    #         Node(
    #             package="controller_manager",
    #             executable="spawner.py",
    #             arguments=["joint_broad"],
    #         )
    #     ],
    # )

    return LaunchDescription([
        control_spawner
    ])
