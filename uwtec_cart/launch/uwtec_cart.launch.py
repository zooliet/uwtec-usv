from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # initialize Arguments
    gui = LaunchConfiguration("gui")

    # initialize file paths
    ros2_controller_file_path = PathJoinSubstitution(
        [
            FindPackageShare("uwtec_cart"),
            "config",
            "uwtec_cart_controller.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("uwtec_cart"), "rviz", "lab.071.uwtec_cart.rviz"]
    )

    # get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("uwtec_cart"),
                    "urdf",
                    "uwtec_cart_ros2_control.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # define nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controller_file_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--param-file",
            ros2_controller_file_path,
            # "--controller-ros-args",
            # "-r /diff_drive_base_controller/cmd_vel:=/cmd_vel",
        ],
    )

    # delay rviz_node after joint_state_broadcaster_spawner
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # delay robot_controller after joint_state_broadcaster_spawner
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    # create launch description and populate
    return LaunchDescription(declared_arguments + nodes)
