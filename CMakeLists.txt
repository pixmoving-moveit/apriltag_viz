cmake_minimum_required(VERSION 3.5)
project(apriltag2_viz)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_directories(${OpenCV_LINK_DIRECTORIES})

add_executable(apriltag2_viz src/apriltag2_viz.cpp)
ament_target_dependencies(apriltag2_viz
  rclcpp
  sensor_msgs
  apriltag_msgs)
target_link_libraries(apriltag2_viz ${OpenCV_LIBRARIES})

install(TARGETS apriltag2_viz DESTINATION lib/${PROJECT_NAME})

ament_package()
