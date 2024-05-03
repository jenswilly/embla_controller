# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# TODO: Clean up and reorganize. This file is a bit messy.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="embla.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    # Initialize Arguments
    description_file = LaunchConfiguration("description_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("embla_controller"), "urdf", description_file]
            ),
        ]
    )
    robot_description = {'robot_description': ParameterValue( robot_description_content, value_type=str) }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("embla_controller"), "rviz", "embla_view.rviz"]
    )

    # Joint state publisher GUI
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Rviz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)