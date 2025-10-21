cmake_minimum_required(VERSION 3.10)
project(lidar_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

set(ALL_TARGET_LIBRARIES "")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

if(glog_FOUND)
  message(STATUS "glog found: ${glog_DIR}")
else()
  message(FATAL_ERROR "glog is not found")
endif()

find_package(rosidl_default_generators REQUIRED)

include_directories(
  include 
  $(EIGEN3_INCLUDE_DIR)
  )
include(cmake/global_defination.cmake)
include(cmake/geographic.cmake)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(test_frame_node src/apps/test_frame_node.cpp ${ALL_SRCS})

ament_target_dependencies(test_frame_node 
  rclcpp 
  std_msgs 
  nav_msgs
  sensor_msgs 
  pcl_conversions 
  pcl_ros 
  tf2 
  tf2_ros 
  Eigen3)

target_link_libraries(test_frame_node glog)
target_link_libraries(test_frame_node ${ALL_TARGET_LIBRARIES})

install(TARGETS test_frame_node DESTINATION lib/${PROJECT_NAME})

#Launch files 
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_dependencies(rclcpp 
  std_msgs 
  nav_msgs
  sensor_msgs 
  pcl_conversions 
  pcl_ros 
  tf2 
  tf2_ros 
  Eigen3)

ament_package()
