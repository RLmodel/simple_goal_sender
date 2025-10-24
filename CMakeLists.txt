cmake_minimum_required(VERSION 3.0.2)
project(movebase_goal_sender)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  move_base_msgs
  geometry_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
