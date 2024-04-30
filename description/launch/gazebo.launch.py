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
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from os import pathsep

def generate_launch_description():
    # Declare arguments
    model_arg = DeclareLaunchArgument(
        "description_file",
        default_value="embla_description.urdf.xacro",
        description="URDF/XACRO description file with the robot.",
    )

    # Initialize Arguments
    description_file = LaunchConfiguration("description_file")

    # Paths
    share_dir_path = get_package_share_directory("embla_controller")
    description_file_path = PathJoinSubstitution([share_dir_path, "urdf", description_file])
    package_prefix_path = get_package_prefix("embla_controller")

    models_path = os.path.join(share_dir_path, "models")
    models_path += pathsep + os.path.join(package_prefix_path, "share")

    # ENV Variables
    env_vars = SetEnvironmentVariable("GAZEBO_MODEL_PATH", models_path)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file_path,
        ]
    )
    robot_description = {'robot_description': ParameterValue( robot_description_content, value_type=str) }

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

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"
    ])))

    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"
    ])))

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "embla", "-topic", "robot_description"],
        output="screen",
    )

    return LaunchDescription([
        env_vars,
        model_arg,
        robot_state_publisher_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
    ])