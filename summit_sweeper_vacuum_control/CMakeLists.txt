cmake_minimum_required(VERSION 3.0.2)
project(summit_sweeper_vacuum_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/vacuum_control_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
