# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, AndSubstitution, NotSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_loc",
            default_value="false",
            description="Launch robot_localization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "playback",
            default_value="false",
            description="Use data from ros bag play.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_robot_localization = LaunchConfiguration("robot_loc")
    use_playback = LaunchConfiguration("playback")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("embla_controller"), "urdf", "embla.urdf.xacro"]
            ),
        ]
    )
    robot_description = {'robot_description': ParameterValue( robot_description_content, value_type=str) }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("embla_controller"), "rviz", "embla_view.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/embla_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    #    condition=UnlessCondition(use_playback),
    )

    # controller_manager
    controller_config_file = PathJoinSubstitution([FindPackageShare("embla_controller"), "config", "embla_controllers.yaml"])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        emulate_tty=True,   # Because we want color output
        condition=IfCondition(AndSubstitution(use_robot_localization, NotSubstitution(use_playback)))
    )

    # controller_manager publishing odometry to be used when _not_ using robot_localization
    controller_config_file_with_odom = PathJoinSubstitution([FindPackageShare("embla_controller"), "config", "embla_controllers_odom.yaml"])
    control_node_odom = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file_with_odom],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        emulate_tty=True,   # Because we want color output
        condition=IfCondition(AndSubstitution(NotSubstitution(use_robot_localization), NotSubstitution(use_playback)))
    )

    # robot_localization
    robot_localization_config_file = PathJoinSubstitution([FindPackageShare("embla_controller"), "config", "ekf_test.yaml"])
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        output="both",
        parameters=[robot_localization_config_file],
        condition=IfCondition(use_robot_localization),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #    condition=UnlessCondition(use_playback),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["embla_base_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_playback),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        control_node_odom,
        robot_localization_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
