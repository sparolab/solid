cmake_minimum_required(VERSION 3.0.2)
project(solid)

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  roscpp
  rospy
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(solid src/solid.cpp)
target_link_libraries(solid ${PCL_LIBRARIES})

add_executable(test_solid src/test.cpp)
target_link_libraries(test_solid solid)
