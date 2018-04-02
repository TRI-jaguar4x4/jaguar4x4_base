cmake_minimum_required(VERSION 3.5)
project(jaguar4x4_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jaguar4x4_base_msgs REQUIRED)

include_directories(include)

add_library(jaguar4x4_base_communication src/Communication.cpp src/BaseReceive.cpp src/BaseCommand.cpp)
add_library(jaguar4x4_motor_parse src/MotorParse.cpp)
add_library(jaguar4x4_utils src/Utils.cpp)
add_executable(jaguar4x4_base_node src/jaguar4x4_base.cpp)
target_link_libraries(jaguar4x4_base_node jaguar4x4_base_communication jaguar4x4_motor_parse jaguar4x4_utils)
ament_target_dependencies(jaguar4x4_base_node rclcpp std_msgs sensor_msgs geometry_msgs jaguar4x4_base_msgs Boost)

install(TARGETS
  jaguar4x4_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  jaguar4x4_base_communication
  jaguar4x4_motor_parse
  jaguar4x4_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_package()
