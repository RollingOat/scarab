cmake_minimum_required(VERSION 2.8.3)
project(dynamic_logistics_warehouse)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)
# Add the source files for the executable
add_executable(moving_object src/object_mover.cpp)

target_link_libraries(moving_object
  ${catkin_LIBRARIES}
)



