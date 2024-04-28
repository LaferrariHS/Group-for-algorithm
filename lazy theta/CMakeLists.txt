cmake_minimum_required(VERSION 3.0.2)
project(graph_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  global_planner
  utils
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS global_planner utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/graph_planner.cpp
  src/lazy_theta_star.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
