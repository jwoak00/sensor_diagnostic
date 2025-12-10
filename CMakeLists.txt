cmake_minimum_required(VERSION 3.8)
project(sensor_diagnostic)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(lidar_diagnostic SHARED
  src/lidar_diagnostic.cpp
)

rclcpp_components_register_node(lidar_diagnostic
  PLUGIN "sensor_diagnostic::LidarDiagnostic"
  EXECUTABLE lidar_diagnostic_node
)

ament_auto_add_library(imu_diagnostic SHARED
  src/imu_diagnostic.cpp
)

rclcpp_components_register_node(imu_diagnostic
  PLUGIN "sensor_diagnostic::ImuDiagnostic"
  EXECUTABLE imu_diagnostic_node
)

ament_auto_add_library(camera_diagnostic SHARED
  src/camera_diagnostic.cpp
)

rclcpp_components_register_node(camera_diagnostic
  PLUGIN "sensor_diagnostic::CameraDiagnostic"
  EXECUTABLE camera_diagnostic_node
)

ament_auto_add_library(gnss_diagnostic SHARED
  src/gnss_diagnostic.cpp
)

rclcpp_components_register_node(gnss_diagnostic
  PLUGIN "sensor_diagnostic::GnssDiagnostic"
  EXECUTABLE gnss_diagnostic_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  # config
)
