cmake_minimum_required(VERSION 2.8.3)
project(simple_local_planner)

find_package(catkin REQUIRED  COMPONENTS geometry_msgs tf cmake_modules costmap_2d nav_core nav_msgs pluginlib roscpp)


catkin_package(LIBRARIES simple_local_planner CATKIN_DEPENDS pluginlib roscpp)

include_directories(${catkin_INCLUDE_DIRS})

add_library(simple_local_planner src/simple_local_planner.cpp)

target_link_libraries(simple_local_planner ${catkin_LIBRARIES})
add_dependencies(simple_local_planner nav_msgs_gencpp)

