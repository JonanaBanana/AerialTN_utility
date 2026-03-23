cmake_minimum_required(VERSION 3.8)
project(aerial_tn_utility)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC  REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL   REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(SWSCALE  REQUIRED IMPORTED_TARGET libswscale)

add_executable(px4_odom_republisher_node src/px4_odom_republisher_node.cpp)
ament_target_dependencies(px4_odom_republisher_node rclcpp px4_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS px4_odom_republisher_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
