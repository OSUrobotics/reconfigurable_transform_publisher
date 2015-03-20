cmake_minimum_required(VERSION 2.8.3)
project(reconfigurable_transform_publisher)
find_package(catkin REQUIRED COMPONENTS rospy tf dynamic_reconfigure)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/Transform.cfg)

catkin_package(CATKIN_DEPENDS rospy tf dynamic_reconfigure)

install(DIRECTORY nodes/ cfg/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS nodes/transform_publisher.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})