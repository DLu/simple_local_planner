cmake_minimum_required(VERSION 2.8.3)
project(simple_local_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_local_planner
  src/${PROJECT_NAME}/simple_local_planner.cpp
)


