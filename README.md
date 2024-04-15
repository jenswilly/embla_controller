# :warning: Work in progress

This is the ROS 2 package for controllers for the JWR-02 Embla differential/skid steering robot.

Don't attempt to build this package yet - it will not work.

Also, there is no documentation ;)

## ROS 2 Control

This package contains:

1. Launch file that launches a `controller_manager/ros2_control_node`, a `robot_state_publisher/robot_state_publisher` and `controller_manager/spawner` for `joint_state_broadcaster` and `joint_state_broadcaster`.
2. Config files
3. URDF
4. Source files for RoboClaw EMCU driver
5. Source files for hardware interface for the JWR-02 Embla system

## Installing

Some Boost libraries are required, so run `sudo apt install libboost-all-dev` first.

Then, as per usual, clone this repo into your workspace's `src` folder and run `colcon build`.
