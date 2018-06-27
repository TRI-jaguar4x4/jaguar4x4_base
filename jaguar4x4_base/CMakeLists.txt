cmake_minimum_required(VERSION 3.5)
project(jaguar4x4_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_mobile_robot REQUIRED)
# FIXME: this should not be required, but somehow the lower layers are not
# exporting this properly so that we can depend on it
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jaguar4x4_base_msgs REQUIRED)
find_package(jaguar4x4_comms REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(jaguar4x4_base_node src/jaguar4x4_base.cpp src/BaseReceive.cpp src/BaseCommand.cpp)
ament_target_dependencies(jaguar4x4_base_node
  geometry_msgs
  jaguar4x4_base_msgs
  jaguar4x4_comms
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
)

target_include_directories(jaguar4x4_base_node
  PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(jaguar4x4_base_node
  ecl_mobile_robot::ecl_mobile_robot
)

install(TARGETS
  jaguar4x4_base_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
