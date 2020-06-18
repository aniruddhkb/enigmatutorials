cmake_minimum_required(VERSION 3.0.2)
project(intro2roscpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hello_world src/hello_world.cpp)
target_link_libraries(hello_world ${catkin_LIBRARIES})

add_executable(chatter_pub src/chatter_pub.cpp)
target_link_libraries(chatter_pub ${catkin_LIBRARIES})

add_executable(chatter_sub src/chatter_sub.cpp)
target_link_libraries(chatter_sub ${catkin_LIBRARIES})