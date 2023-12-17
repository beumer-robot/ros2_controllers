cmake_minimum_required(VERSION 3.5)
project(swerve_steering_controller)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies using ament instead of catkin
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Updated dependency search
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

generate_parameter_library(swerve_steering_controllers_library_parameters
  src/swerve_steering_controllers_library_parameters.yaml
)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Declare a library
add_library(${PROJECT_NAME}
  src/utils.cpp
  src/swerve_steering_controller.cpp
  src/wheel.cpp
  src/interval.cpp
  src/speed_limiter.cpp
  src/odometry.cpp
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  control_msgs
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  tf2_ros
  urdf
)

# Install plugins, headers, and library
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES swerve_steering_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Uncomment if you have tests
# if(BUILD_TESTING)
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()
# endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

# Package configuration
ament_package()
