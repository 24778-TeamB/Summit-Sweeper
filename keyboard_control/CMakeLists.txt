cmake_minimum_required(VERSION 3.0.2)
project(keyboard_control)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(cpp_keyboard src/keyboard_control.cpp)
target_link_libraries(cpp_keyboard ${catkin_LIBRARIES})
catkin_install_python(PROGRAMS
  scripts/py_keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
