cmake_minimum_required(VERSION 2.8.3)
project(multiple_kinect_baxter_calibration)

## Add support for C++11
# add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

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
  ${OpenCV_INCLUDE_DIRS}
)

## Print PCL Version (for debugging only) PCL_VERSION: 1.7.1
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

## Build executables and link against libraries
add_executable(sphere_detector_test
  src/sphere_detector_test.cpp)
target_link_libraries(sphere_detector_test
  ${PROJECT_NAME}_sphere_detector
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(data_collector
  src/data_collector.cpp)
target_link_libraries(data_collector
  ${PROJECT_NAME}_sphere_detector
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(save_pcd
  src/save_pcd.cpp)
target_link_libraries(save_pcd
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(view_cloud_realtime
  src/view_cloud_realtime.cpp)
target_link_libraries(view_cloud_realtime
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(show_all_point_cloud
  src/show_all_point_cloud.cpp)
target_link_libraries(show_all_point_cloud
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(merge_point_clouds
  src/merge_point_clouds.cpp)
target_link_libraries(merge_point_clouds
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(view_image
  src/view_image.cpp)
target_link_libraries(view_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(segment_image
  src/segment_image.cpp)
target_link_libraries(segment_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(view_pcd
  src/view_pcd.cpp)
target_link_libraries(view_pcd
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
