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
#
# Parameters:
# teleop:=true|false        Launch RC teleop nodes (sbus). Default: true
# imu:=true|false           Launch VMU931 IMU nodes. Default: true
# lidar:=true|false         Launch RPLidar node and I2C service. Default: true
# robot_loc:=true|false     Launch robot_localization node for sensor fusion. Default: true
# generate_map:=true|false  Run slam_toolbox in mapper mode. Default: false
# nav2_loc:=true|false      Run nav2 stack for localization with pre-generated map. Default: false
# map_path:=<path>          Path to pre-generated map. Default: "<embla_controller_share_directory>/maps/save.yaml"

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
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop",
            default_value="true",
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_loc",
            default_value="true",
            description="Launch robot_localization.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_map",
            default_value="false",
            description="Run slam_toolbox in mapper mode.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "nav2_loc",
            default_value="false",
            description="Run nav2 stack for localization.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "map_path",
            default_value=PathJoinSubstitution([FindPackageShare("vmu931_imu"), "maps", "save.yaml"]),
            description="Run slam_toolbox in mapper mode.",
        )
    )


    # Initialize Arguments
    use_teleop = LaunchConfiguration("teleop")
    use_imu = LaunchConfiguration("imu")
    use_lidar = LaunchConfiguration("lidar")
    use_robot_localization = LaunchConfiguration("robot_loc")
    generate_map = LaunchConfiguration("generate_map")
    use_nav2 = LaunchConfiguration("nav2_loc")
    map_path = LaunchConfiguration("map_path")

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

    # i2c_service
    i2c_service_node = Node(
        package="i2c_service",
        executable="i2c_service",
        output="both",
        parameters=[{'i2c_bus': 1}],
        condition=IfCondition(use_lidar),   # Only needed if lidar is used
    )

    # RPLidar
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_publisher",
        output="both",
        parameters=[
            {'serial_port': '/dev/ttyAMA1'},
            {'frame_id': 'lidar_link'},
            {'angle_compensate': True},
        ],
        condition=IfCondition(use_lidar),
    )

    # VMU931 IMU
    imu_launch_file = PathJoinSubstitution([FindPackageShare("vmu931_imu"), "launch", "vmu931_imu_launch.xml"])
    imu_launch = IncludeLaunchDescription(
        imu_launch_file,
        condition=IfCondition(use_imu),
    )
    
    # robot_localization
    robot_localization_config_file = PathJoinSubstitution([FindPackageShare("embla_controller"), "config", "ekf.yaml"])
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        output="both",
        parameters=[robot_localization_config_file],
        condition=IfCondition(use_robot_localization),
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
        condition=IfCondition(use_robot_localization),
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
        condition=UnlessCondition(use_robot_localization),
    )

    # robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    # joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # custom controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["embla_base_controller", "--controller-manager", "/controller_manager"],
    )

    # sbus_serial
    sbus_launch_file = PathJoinSubstitution([FindPackageShare("sbus_serial"), "launch", "embla_teleop_launch.yaml"])
    sbus_launch = IncludeLaunchDescription(
        sbus_launch_file,
        condition=IfCondition(use_teleop),
    )

    # Generate map
    # ros2 launch slam_toolbox online_async_launch.py params_file:=./install/embla_controller/share/embla_controller/config/mapper_params_online_async.yaml
    slam_launch_file = PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
    mapper_config_file = PathJoinSubstitution([FindPackageShare("embla_controller"), "config", "mapper_params_online_async.yaml"])
    generate_map_launch =IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'slam_params_file': mapper_config_file}.items(),
        condition=IfCondition(generate_map),
    )

    # Delay slam_toolbox launch after joint_state_broadcaster
    delay_generate_map_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[generate_map_launch],
        )
    )

    # Nav2 localization
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), 'launch', 'localization_launch.py'])
        ),
        launch_arguments={
            'map': map_path,
            'params_file': PathJoinSubstitution([FindPackageShare("embla_controller"), 'config', 'nav2_localization.yaml'])
        }.items(),
        condition=IfCondition(use_nav2),
    )


    nodes = [
        control_node,
        control_node_odom,
        robot_state_pub_node,
        robot_localization_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        sbus_launch,
        i2c_service_node,
        lidar_node,
        imu_launch,
        delay_generate_map_after_joint_state_broadcaster_spawner, # generate_map_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)