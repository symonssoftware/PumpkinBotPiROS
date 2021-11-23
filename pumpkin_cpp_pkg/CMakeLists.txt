cmake_minimum_required(VERSION 3.5)
project(pumpkin_cpp_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros tf2)

add_executable(pumpkin_clock src/pumpkin_clock.cpp)
ament_target_dependencies(pumpkin_clock rclcpp builtin_interfaces)

install(TARGETS
    odometry
    pumpkin_clock
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
