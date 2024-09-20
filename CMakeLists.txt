cmake_minimum_required(VERSION 3.5)
project(marker_to_pointcloud)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(marker_to_pointcloud_component SHARED
  src/marker_to_pointcloud_component.cpp
)

target_include_directories(marker_to_pointcloud_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(marker_to_pointcloud_component
  "rclcpp"
  "rclcpp_components"
  "visualization_msgs"
  "sensor_msgs"
  "tf2_ros"
  "tf2_sensor_msgs"
  "tf2_geometry_msgs"
  "geometry_msgs"
)

rclcpp_components_register_node(marker_to_pointcloud_component
  PLUGIN "marker_to_pointcloud::MarkerToPointCloud"
  EXECUTABLE marker_to_pointcloud_node)

install(TARGETS marker_to_pointcloud_component
  EXPORT export_marker_to_pointcloud
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
ament_package()
