cmake_minimum_required(VERSION 3.0.2)
project(intro2rospy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  turtlesim
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/hello_world.py
  script/random_publisher.py
  script/pose_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)