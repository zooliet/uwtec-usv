from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    RegisterEventHandler,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Flag to use simulation (Gazebo) clock if true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_jsp_gui",
            default_value="false",
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

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         name="use_headless",
    #         default_value="false",
    #         choices=["true", "false"],
    #         description="Flag to enable headless mode with no GUI",
    #     )
    # )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="urdf_file",
            default_value="uwtec_cart_gazebo.urdf.xacro",
            description="URDF file name",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="world_file",
            default_value="empty.sdf",
            description="World file name to load in Gazebo",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="rviz_config_file",
            default_value="uwtec_cart_gazebo.rviz",
            description="RViz configuration file name",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="ros_gz_bridge_file",
            default_value="ros_gz_bridge_params.yaml",
            description="ROS-Gazebo bridge configuration file name",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="ros2_controller_file",
            default_value="uwtec_cart_controller.yaml",
            description="ROS2 controller configuration file name",
        )
    )

    # initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    # use_headless = LaunchConfiguration("use_headless")
    urdf_file = LaunchConfiguration("urdf_file")
    world_file = LaunchConfiguration("world_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ros_gz_bridge_file = LaunchConfiguration("ros_gz_bridge_file")
    ros2_controller_file = LaunchConfiguration("ros2_controller_file")

    # initialize file paths
    world_file_path = PathJoinSubstitution(
        [
            FindPackageShare("uwtec_cart"),
            "worlds",
            world_file,
        ]
    )

    ros_gz_bridge_file_path = PathJoinSubstitution(
        [
            FindPackageShare("uwtec_cart"),
            "config",
            ros_gz_bridge_file,
        ]
    )

    ros2_controller_file_path = PathJoinSubstitution(
        [
            FindPackageShare("uwtec_cart"),
            "config",
            ros2_controller_file,
        ]
    )

    # set environment variables for models
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        PathJoinSubstitution([FindPackageShare("uwtec_cart"), "models"]),
    )

    # define nodes
    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("uwtec_cart"),
                    "launch",
                    "robot_state_publisher.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_jsp_gui": use_jsp_gui,
            "use_rviz": use_rviz,
            # "robot_name": robot_name,
            "urdf_file": urdf_file,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file_path],
            # "gz_args": ["-r -s -v4 ", world_file_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # gazebo_client_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("ros_gz_sim"),
    #                     "launch",
    #                     "gz_sim.launch.py",
    #                 ]
    #             )
    #         ]
    #     ),
    #     launch_arguments={
    #         "gz_args": "-g",
    #     }.items(),
    #     condition=UnlessCondition(use_headless),
    # )

    gazebo_ros_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": ros_gz_bridge_file_path,
            }
        ],
        # arguments=[
        #     "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        # ],
    )

    gazebo_ros_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "uwtec_cart",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "0",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    # joint_state_broadcaster_spawner = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "load_controller",
    #         "--set-state",
    #         "active",
    #         "joint_state_broadcaster",
    #     ],
    #     output="screen",
    # )

    diff_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--param-file",
            ros2_controller_file_path,
        ],
    )
    # diff_drive_base_controller_spawner = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "load_controller",
    #         "--set-state",
    #         "active",
    #         "diff_drive_base_controller",
    #     ],
    #     output="screen",
    # )

    # delay joint_state_broadcaster_spawner after gazebo_ros_spawner
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_ros_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # delay diff_drive_base_controller_spawner after joint_state_broadcaster_spawner
    delay_diff_drive_base_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_base_controller_spawner],
        )
    )

    nodes = [
        set_env_vars_resources,
        robot_state_publisher_node,
        gazebo_node,
        # gazebo_client_node,
        gazebo_ros_bridge_node,
        gazebo_ros_spawner,
        delay_joint_state_broadcaster_spawner,
        delay_diff_drive_base_controller_spawner,
    ]

    # create launch description and populate
    return LaunchDescription(declared_arguments + nodes)
