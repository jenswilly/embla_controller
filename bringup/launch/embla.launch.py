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
# This file is the main bringup launch file for the Embla JWR-02 robot.
# It launches the controller_manager, robot_state_publisher, joint_state_broadcaster, and the base controller.
# It also launches the sbus_serial node for remote control teleoperation if the `teleop` argument is set to true.
# ------------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop",
            default_value="false",
            description="Also launch RC teleop nodes (sbus).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "imu",
            default_value="true",
            description="Launch VMU931 IMU.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar",
            default_value="true",
            description="Launch RPLidar.",
        )
    )

    # Initialize Arguments
    use_teleop = LaunchConfiguration("teleop")
    use_imu = LaunchConfiguration("imu")
    use_lidar = LaunchConfiguration("lidar")

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

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("embla_controller"),
            "config",
            "embla_controllers.yaml",
        ]
    )

    i2c_service_node = Node(
        package="i2c_service",
        executable="i2c_service",
        output="both",
        parameters=[{'i2c_bus': 1}],
    )

    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_publisher",
        output="both",
        parameters=[
            {'serial_port': '/dev/ttyAMA1'},
            {'frame_id': 'laser_frame'},
        ],
        condition=IfCondition(use_lidar),
    )

    imu_launch_file = PathJoinSubstitution(   
                [FindPackageShare("vmu931_imu"), "launch", "vmu931_imu_launch.xml"]
            )
    imu_node = IncludeLaunchDescription(
        imu_launch_file,
        condition=IfCondition(use_imu),
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        emulate_tty=True,   # Because we want color output
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["embla_base_controller", "--controller-manager", "/controller_manager"],
    )

    sbus_launch_file = PathJoinSubstitution(   
                [FindPackageShare("sbus_serial"), "launch", "embla_teleop_launch.yaml"]
            )
    sbus_node = IncludeLaunchDescription(
        sbus_launch_file,
        condition=IfCondition(use_teleop),
    )

    delay_joint_state_broadcaster_after_rplidar = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=lidar_node,
            on_exit=[joint_state_broadcaster_spawner],
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
        sbus_node,
        i2c_service_node,
        lidar_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner, # delay_joint_state_broadcaster_after_rplidar,
        robot_controller_spawner, # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)