# Install script for directory: /home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/src/intro2rospy

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/build/intro2rospy/catkin_generated/installspace/intro2rospy.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intro2rospy/cmake" TYPE FILE FILES
    "/home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/build/intro2rospy/catkin_generated/installspace/intro2rospyConfig.cmake"
    "/home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/build/intro2rospy/catkin_generated/installspace/intro2rospyConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intro2rospy" TYPE FILE FILES "/home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/src/intro2rospy/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/intro2rospy" TYPE PROGRAM FILES "/home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/build/intro2rospy/catkin_generated/installspace/hello_world.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/intro2rospy" TYPE PROGRAM FILES "/home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/build/intro2rospy/catkin_generated/installspace/random_publisher.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/intro2rospy" TYPE PROGRAM FILES "/home/akb/Documents/github/enigmatutorials/intro2ros/workspaces/intro2ros/build/intro2rospy/catkin_generated/installspace/pose_subscriber.py")
endif()

