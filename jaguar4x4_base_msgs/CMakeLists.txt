cmake_minimum_required(VERSION 3.5)
project(jaguar4x4_base_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jaguar4x4_comms_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(jaguar4x4_base_msgs
  msg/Motors.msg
  srv/BaseRPMCalculator.srv
  DEPENDENCIES builtin_interfaces std_msgs jaguar4x4_comms_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
