cmake_minimum_required(VERSION 2.8.3)
project(mesh80211s)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs)

add_message_files(FILES MeshStation.msg MeshStations.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBNL3 libnl-3 REQUIRED)
#find_package(LIBNL-3 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
find_package(LibNL REQUIRED)
include_directories(${LibNL_INCLUDE_DIR})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/mesh80211s.cc src/station.c)
target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${LibNL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
