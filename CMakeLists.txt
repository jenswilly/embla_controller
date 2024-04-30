cmake_minimum_required(VERSION 3.8)
project(embla_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# List of dependencies
set(DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Boost
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono system) # Boost::chrono is used by TimeoutSerial

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/embla_system.cpp
  hardware/roboclaw/roboclaw_driver.cpp
  hardware/roboclaw/TimeoutSerial.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-psabi) # Ignore ABI warnings
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(${PROJECT_NAME} PRIVATE "EMBLA_CONTROLLER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface embla_controller.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY 
    description/launch 
    description/ros2_control 
    description/urdf
    description/models
    bringup/launch
    bringup/config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()