cmake_minimum_required(VERSION 2.8.3)
project(cameras)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add image saver class node
add_executable(image_saver src/image_saver.cpp)
target_link_libraries(image_saver ${catkin_LIBRARIES})
add_dependencies(image_saver ${catkin_EXPORTED_TARGETS})
