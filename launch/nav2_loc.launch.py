# The MIT License (MIT)
# 
# Copyright (c) 2024 Jens Willy Johannsen <jens@jwrobotics.com>
# 
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# ------------------------------------------------------------------------------
# This file launches nav2 localization with pre-generated map.
# This functionality can also be run from the main bringup launch file by specifying `nav2_loc:=true`.
#
# Parameters:
# map_path:=<path>      Path to pre-generated map. Default: "<embla_controller_share_directory>/maps/save.yaml"
# config_file:=<path>   Path to nav2 config file. Default: "<embla_controller_share_directory>/config/nav2_localization.yaml"

# ------------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=PathJoinSubstitution([FindPackageShare("embla_controller"), "maps", "save.yaml"]),
        description="Path to pre-generated map YAML file for nav2 localization.",
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([FindPackageShare("embla_controller"), "config", "nav2_localization.yaml"]),
        description="Path to nav2 parameters file",
    )

    # Nav2 localization
    config_file_path = LaunchConfiguration("config_file")
    map_path = LaunchConfiguration("map_path")

    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), 'launch', 'localization_launch.py'])
        ),
        launch_arguments={
            'map': map_path,
            'params_file': config_file_path,
        }.items(),
    )

    return LaunchDescription([map_path_arg, config_file_arg, nav2_localization_launch])