cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(cameras)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  cables_detection
  cv_bridge
  fiducial_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
  Obj3D_Poses.srv
  Aruco3D_Pose.srv
  Aruco2D_Poses.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_saver
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  image_saver
  src/image_saver.cpp
)

# Add image saver class node
add_executable(image_saver_node src/image_saver_node.cpp)
target_link_libraries(image_saver_node image_saver ${catkin_LIBRARIES})
# add_dependencies(image_saver ${catkin_EXPORTED_TARGETS})
