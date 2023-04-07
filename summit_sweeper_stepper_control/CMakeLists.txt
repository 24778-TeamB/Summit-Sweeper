cmake_minimum_required(VERSION 3.0.2)
project(summit_sweeper_stepper_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/vertical_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
