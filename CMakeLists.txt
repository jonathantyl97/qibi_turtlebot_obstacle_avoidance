cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  tf
  move_base_msgs
  actionlib
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/turtlebot3_obstacle_avoidance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
