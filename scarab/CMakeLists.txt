cmake_minimum_required(VERSION 2.8.3)
project(scarab)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY maps/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

catkin_install_python(PROGRAMS scripts/odom_to_tf.py src/camera_tf_broadcaster.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})