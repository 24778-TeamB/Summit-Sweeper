cmake_minimum_required(VERSION 3.0.2)
project(summit_sweeper_sensors)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/distance_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
