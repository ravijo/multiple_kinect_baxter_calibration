cmake_minimum_required(VERSION 2.8.3)
project(multiple_kinect_baxter_calibration)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS ...
#  LIBRARIES ...
#  CATKIN_DEPENDS
  DEPENDS Eigen
)

## Build
include_directories(
  include/${PROJECT_NAME}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Print PCL Version (for debugging only)
MESSAGE(STATUS "PCL_VERSION: " ${PCL_VERSION})

## Declare sphere_detector as a library
add_library(${PROJECT_NAME}_sphere_detector
  src/sphere_detector.cpp
)

## Link sphere_detector library against PCL and catkin
target_link_libraries(${PROJECT_NAME}_sphere_detector
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare executables and link against libraries
add_executable(sphere_detector_test src/sphere_detector_test.cpp)
target_link_libraries(sphere_detector_test
  ${PROJECT_NAME}_sphere_detector
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(data_collector src/data_collector.cpp)
target_link_libraries(data_collector
  ${PROJECT_NAME}_sphere_detector
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(save_pcd src/save_pcd.cpp)
target_link_libraries(save_pcd
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(view_cloud_realtime src/view_cloud_realtime.cpp)
target_link_libraries(view_cloud_realtime
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
