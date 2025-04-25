cmake_minimum_required(VERSION 3.8)
project(pov_bluetooth_central)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/bluetooth_central_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}/${PROJECT_NAME}
)

ament_package()