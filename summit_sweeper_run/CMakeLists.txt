cmake_minimum_required(VERSION 3.0.2)
project(summit_sweeper_run)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/summit_sweeper.py
  scripts/test_step.py
  scripts/test_horizontal.py
  scripts/horizontal_speed.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
