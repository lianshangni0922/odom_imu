cmake_minimum_required(VERSION 2.8.3)
project(odom_predictor)

#find_package(catkin REQUIRED)
#catkin_simple(ALL_DEPS_REQUIRED)

#add_definitions(-std=gnu++11 -o3)
include_directories( "/usr/include/eigen3" )
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  serial
  std_msgs
  tf
)

catkin_package(
)

#############
# LIBRARIES #
#############
#add_library(${PROJECT_NAME}
# src/odom_predictor.cpp
#)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
############
# BINARIES #
############
add_executable(${PROJECT_NAME}
  src/odom_predictor.cpp
)
#target_link_libraries(odom_predictor_node ${PROJECT_NAME})

##########
# EXPORT #
##########
#cs_install()
#cs_export()
