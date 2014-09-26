cmake_minimum_required(VERSION 2.8.3)
project(comp417_fall2014_assignment_1_mcl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  occupancy_grid_utils
)

find_package(Eigen REQUIRED) # for Ubuntu < 14.04
#find_package(Eigen3 REQUIRED) # for Ubuntu 14.04

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES comp417_fall2014_assignment_1_mcl
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf occupancy_grid_utils stage stage_ros rviz turtlesim map_server
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(mcllab src/probrob_mcl_node.cpp src/MCLocaliser.cpp)
add_dependencies(mcllab std_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
target_link_libraries(mcllab ${catkin_LIBRARIES})
