cmake_minimum_required(VERSION 2.8.3)
project(hfn)

set(CMAKE_CXX_FLAGS "-frounding-math ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  angles
  scarab_msgs
)

generate_dynamic_reconfigure_options(cfg/HumanFriendlyNavigation.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES playermap
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    angles
    scarab_msgs
  DEPENDS CGAL Eigen3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})

add_library(playermap src/map.c src/rosmap.cpp)
target_link_libraries(playermap ${catkin_LIBRARIES})

add_library(hfnlib src/hfn.cpp)
target_link_libraries(hfnlib playermap ${catkin_LIBRARIES} ${CGAL_LIBRARY} ${GMP_LIBRARIES})
add_dependencies(hfnlib ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(hfn src/hfn_node.cpp)
target_link_libraries(hfn ${catkin_LIBRARIES} hfnlib playermap)
add_dependencies(hfn ${PROJECT_NAME}_gencfg)

add_executable(scarab_wp_node src/scarab_wp_node.cpp)
target_link_libraries(scarab_wp_node ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})
add_dependencies(scarab_wp_node scarab_wp_node_generate_messages_cpp)

install(TARGETS
    playermap
    hfn
    hfnlib
    scarab_wp_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS scripts/goal_to_action.py scripts/tf_posestamped_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
