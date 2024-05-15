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
# This file launches nav2 navigation.

# A `twist_stamper` node is also launched to remap the cmd_vel topic from the nav2 controllers to
# the `/embla_base_controller/cmd_vel` topic and convert to `TwistStamped` messages.
#
# Parameters:
# config_file:=<path>   Path to nav2 config file. Default: "<embla_controller_share_directory>/config/nav2_navigation.yaml"

# ------------------------------------------------------------------------------
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

    # We need to "stamp" the cmd_vel messages from the nav2 controllers
    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        output="screen",
        remappings=[
            ('cmd_vel_in', 'cmd_vel'),
            ('cmd_vel_out', '/embla_base_controller/cmd_vel'),
        ]
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
                  'use_composition': 'False',
            }.items()
        ),
    ])

    return LaunchDescription([config_file_arg, twist_stamper_node, nav2])
