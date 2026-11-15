
cmake_minimum_required(VERSION 3.8)
project(rov_control)

if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rov_interface REQUIRED)

# Main controller
add_executable(controller src/controller.cpp)

ament_target_dependencies(controller
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  rov_interface
)

add_executable(simulator src/simulator.cpp)

ament_target_dependencies(simulator
  rclcpp
  std_msgs
)

add_executable(cancel src/cancel.cpp)

ament_target_dependencies(cancel
  rclcpp
  rclcpp_action
  rov_interface
)

install(TARGETS
  controller
  simulator
  cancel
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
