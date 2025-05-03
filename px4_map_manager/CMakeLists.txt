cmake_minimum_required(VERSION 3.5)
project(px4_map_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)


# Include headers from include/
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckPosCollision.srv"
  "srv/RayCast.srv"
  DEPENDENCIES geometry_msgs
)

# occupancy_map node (occupancy grid generator)
add_executable(occupancy_map_node src/occupancy_map_node.cpp src/occupancy_map.cpp include/px4_map_manager/raycast.cpp)
ament_target_dependencies(occupancy_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  cv_bridge
  pcl_conversions
  rosidl_default_runtime
)
rosidl_target_interfaces(occupancy_map_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS occupancy_map_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
