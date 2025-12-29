from launch import LaunchDescription

from launch_ros.actions import SetRemap
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
    GroupAction,
)

# from launch.conditions import IfCondition
# from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    # Command,
    # FindExecutable,
    PathJoinSubstitution,
    # LaunchConfiguration,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # declare arguments
    declared_arguments = []

    # initialize arguments

    # initialize file paths
    robot_localization_params_file = PathJoinSubstitution(
        [
            FindPackageShare("uwtec_nav"),
            "config",
            "dual_ekf_navsat_params.yaml",
        ]
    )

    nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare("uwtec_nav"),
            "config",
            "nav2_no_map_params.yaml",
        ]
    )

    # define nodes
    efk_odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[robot_localization_params_file],
        # remappings=[("odometry/filtered", "odometry/local")],
    )

    efk_map_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[robot_localization_params_file],
        # remappings=[("odometry/filtered", "odometry/global")],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[robot_localization_params_file],
        # remappings=[
        #     ("imu/data", "imu/data"),
        #     ("gps/fix", "gps/fix"),
        #     ("gps/filtered", "gps/filtered"),
        #     ("odometry/gps", "odometry/gps"),
        #     ("odometry/filtered", "odometry/global"),
        # ],
    )

    nodes = [
        efk_odom_node,
        # efk_map_node,
        # navsat_transform_node,
    ]

    # create launch description and populate
    return LaunchDescription(declared_arguments + nodes)


