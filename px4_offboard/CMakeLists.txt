cmake_minimum_required(VERSION 3.5)
project(px4_offboard)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  offboard_control
  visualizer
  velocity_control
  control
  processes
  swarm_processes
  teleop_swarm
  swarm_auto
  DESTINATION lib/${PROJECT_NAME}
)

# Launch and resource files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
