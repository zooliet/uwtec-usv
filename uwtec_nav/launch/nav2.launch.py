
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

    # define nodes
    simple_nav_node = Node(
        package="uwtec_nav",
        executable="simple_nav",
        name="simple_nav",
        output="screen",
    )


    nodes = [
        simple_nav_node
    ]

    # create launch description and populate
    return LaunchDescription(declared_arguments + nodes)



