# Description: Launch file for nav2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([FindPackageShare("embla_controller"), "config", "nav2_navigation.yaml"]),
        description="Path to nav2 parameters file",
    )

    config_file = LaunchConfiguration("config_file")
    nav2 = GroupAction([
        SetRemap('/global_costmap/scan', '/scan'),
        SetRemap('/local_costmap/scan', '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("nav2_bringup"), 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                  'params_file': config_file,
                  'use_composition': False,
            }.items()
        ),
    ])

    return LaunchDescription([config_file_arg, nav2])
