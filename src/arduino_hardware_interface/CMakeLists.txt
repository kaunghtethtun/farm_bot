cmake_minimum_required(VERSION 3.8)
project(arduino_hardware_interface)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# These are not needed for building the hardware interface library,
# but are needed for usage and launching
find_package(controller_manager REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(ros2_control REQUIRED)
find_package(xacro REQUIRED)

# Plugin library
add_library(arduino_system SHARED
  src/arduino_system.cpp
)

target_include_directories(arduino_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arduino_system 
  hardware_interface 
  rclcpp 
  pluginlib
  rclcpp_lifecycle
)

# Export plugin info

ament_export_targets(export_arduino_system HAS_LIBRARY_TARGET)
ament_export_libraries(arduino_system)
ament_export_include_directories(include)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install shared files
install(
  DIRECTORY launch config description
  DESTINATION share/${PROJECT_NAME}
)

# Install library
install(
  TARGETS arduino_system
  EXPORT export_arduino_system
  LIBRARY DESTINATION lib
)

# Lint and tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(hardware_interface arduino_hardware_interface.xml)
ament_package()

