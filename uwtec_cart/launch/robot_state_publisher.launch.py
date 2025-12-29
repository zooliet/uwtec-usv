from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Flag to use simulation (Gazebo) clock if true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_jsp_gui",
            default_value="true",
            choices=["true", "false"],
            description="Flag to enable joint state publisher GUI",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Flag to enable RViz",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="urdf_file",
            # default_value="lab.011.uwtec_cart.urdf.xacro",
            default_value="uwtec_cart.urdf.xacro",
            description="URDF file name",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="rviz_config_file",
            default_value="uwtec_cart_basic.rviz",
            description="RViz configuration file name",
        )
    )

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         name="robot_name",
    #         default_value="uwtec_cart",
    #         description="Name of UWTEC's 1st-gen cart robot",
    #     )
    # )

    # initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    urdf_file = LaunchConfiguration("urdf_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    # robot_name = LaunchConfiguration("robot_name")

    # initialize file paths
    urdf_file_path = PathJoinSubstitution(
        [
            FindPackageShare("uwtec_cart"),
            "urdf",
            urdf_file,
        ]
    )

    rviz_config_file_path = PathJoinSubstitution(
        [
            FindPackageShare("uwtec_cart"),
            "rviz",
            rviz_config_file,
        ]
    )

    # get URDF via xacro
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file_path,
            " ",
            "sim_mode:=",
            use_sim_time,
        ]
    )

    # define nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_jsp_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config_file_path,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    # create launch description and populate
    return LaunchDescription(declared_arguments + nodes)

