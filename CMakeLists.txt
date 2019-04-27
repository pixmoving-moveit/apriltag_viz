cmake_minimum_required(VERSION 3.5)
project(apriltag2_viz)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)

add_library(apriltag2_viz SHARED src/apriltag2_viz.cpp)
ament_target_dependencies(apriltag2_viz
  rclcpp
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  apriltag_msgs)
rclcpp_components_register_nodes(apriltag2_viz "AprilVizNode")

install(TARGETS apriltag2_viz
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
