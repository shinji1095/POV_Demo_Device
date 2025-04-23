cmake_minimum_required(VERSION 3.8)
project(pov_hailo8_node)

# Set C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(HailoRT REQUIRED)
find_package(Threads REQUIRED)
find_package(pov_demo_msg REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # Points to pov_hailo8_node/include
  ${OpenCV_INCLUDE_DIRS}
  ${HailoRT_INCLUDE_DIRS}
)

# Create executable
add_executable(${PROJECT_NAME} src/pov_hailo8_node.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  HailoRT::libhailort
  Threads::Threads
)

# Specify ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  pov_demo_msg
)

# Install the executable
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (optional, if needed for other packages)
install(DIRECTORY include/
  DESTINATION include/
)

# Package configuration
ament_package()