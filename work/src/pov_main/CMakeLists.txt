cmake_minimum_required(VERSION 3.8)
project(pov_main)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pov_demo_msg REQUIRED)

include_directories(
  include
)

add_executable(pov_main_node src/pov_main_node.cpp)

ament_target_dependencies(pov_main_node
  rclcpp
  std_msgs
  pov_demo_msg
)

add_executable(pov_main_test src/pov_main_test.cpp)

ament_target_dependencies(pov_main_test
  rclcpp
  std_msgs
  pov_demo_msg
)


install(TARGETS
  pov_main_node
  pov_main_test 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()