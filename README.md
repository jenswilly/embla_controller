# JWR-02 Embla Controller

The **JWR-02 Embla** robot is a ROS2-controlled robot with the following hardware:

- Raspberry Pi 4 as the main controller
- Custom Auxiliary Interface Board with USB ports, SBUS ↔︎ UART conversion, PWM controller and status LEDs
- RoboClaw 2 × 15 DC motor driver
- Slamtec RPLidar A2
- Variense VMU931 IMU
- UBEC for 3S LiPo battery → Raspberry Pi power

## Contents

This package contains:

1. Hardware interface for the RoboClaw motor controller exposing command and state interfaces
2. URDF description of the bot
3. Config files for ros2_control, robot_localization and slam_toolbox
4. Main bringup launch file launching ros2_control, lidar, IMU, RC teleop, robot_localization and slam_toolbox nodes

### Launch files

#### embla.launch.py

Main bringup launch file. This should be run on the Raspberry Pi.

The following parameters are supported:

| Parameter    | Description                                        | Default           |
| ------------ | -------------------------------------------------- | ----------------- |
| teleop       | Launch RC teleop nodes (sbus)                      | ✅ true           |
| imu          | Launch VMU931 IMU nodes                            | ✅ true           |
| lidar        | Launch RPLidar node and I2C service                | ✅ true           |
| robot_loc    | Launch robot_localization node for IMU/odom fusion | ✅ true           |
| generate_map | Run slam_toolbox in mapper mode                    | **❌ false**      |
| nav2_loc     | Use nav2 for localization with pre-generated map   | **❌ false**      |
| map_path     | Path to map YAML file for nav2 localization        | 📂 maps/save.yaml |

Default (no parameters specified) is to run the bot _with_ teleop and all sensors but no nav2 localization and no slam_toolbox mapping.

Do not use both `generate_map` and `nav2_loc` at the same time.

#### nav2_loc.launch.py

Launches only the localization part of the nav2 stack.

This is the same as specifying `nav2_loc:=true` for the main bringup file but with this launch file
you can also specify a custom nav2 parameters file.

| Parameter   | Description                                 | Default                          |
| ----------- | ------------------------------------------- | -------------------------------- |
| map_path    | Path to map YAML file for nav2 localization | 📂 maps/save.yaml                |
| config_file | Path to map YAML file for nav2 localization | 📂 config/nav2_localization.yaml |

#### nav2_nav.launch.py

Launches the _navigation_ part of the nav2 stack.

A `twist_stamper` node is also launched to remap the unstamped twist messages on `/cmd_vel` to
stamped messages on `/embla_base_controller/cmd_vel`.

| Parameter   | Description                                 | Default                        |
| ----------- | ------------------------------------------- | ------------------------------ |
| config_file | Path to map YAML file for nav2 localization | 📂 config/nav2_navigation.yaml |

#### rviz.launch.py

Launches Rviz2 pre-configured (using `/rviz/embla.rviz`) to show the robot along with IMU and lidar data and the map.  
Navigation/planning are also shown.

This should be launched on a remote computer to monitor the robot.

## Dependencies

This package depends on the following external packages (all available on Github):

- [i2c_service](https://github.com/jenswilly/i2c_service): Service to control the Raspberry Pi's I2C interface. Used for setting Lidar rotation speed via a PWM controller.
- [i2c_interfaces](https://github.com/jenswilly/i2c_interfaces): Interfaces for above package.
- [vmu931_imu](https://github.com/jenswilly/vmu931_imu): Node for the Variense VMU931 IMU.
- [vmu931_imu_interfaces](https://github.com/jenswilly/vmu931_imu_interfaces): Interfaces for above.
- [sbus_serial](https://github.com/jenswilly/sbus_serial): Node and interfaces for using a SBUS-based RC receiver for teleoperation.
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox): For SLAM and map generation
- [nav2_bringup](https://github.com/ros-navigation/navigation2/tree/main/nav2_bringup): Nav2 stack bringup utilities and configurations
- [nav2](https://github.com/ros-navigation/navigation2/tree/main): Nav2 stack
- [twist_stamper](https://github.com/joshnewans/twist_stamper.git): Used to convert from `Twist` messages from nav2 to `TwistStamped` for the controller input.

## Installing

Some Boost libraries are required for the hardware interface, so run `sudo apt install libboost-all-dev` first.

Then install the required dependencies.

Finally, as per usual, clone this repo into your workspace's `src` folder and run `colcon build`.
