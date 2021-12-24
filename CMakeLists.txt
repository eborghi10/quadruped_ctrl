cmake_minimum_required(VERSION 3.0.2)
project(quadruped_ctrl)

set(CMAKE_CXX_FLAGS "-std=c++14 -Werror -Wall -Wextra" CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE "Release")

find_package (Eigen3 REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    commandDes.msg
)

add_service_files(
  FILES
    QuadrupedCmd.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    std_msgs
    message_runtime
    geometry_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS}
  include
  include/${PROJECT_NAME}
  ${Eigen3_INCLUDE_DIRS}
  src
)

add_subdirectory(src/qpOASES)
add_subdirectory(src/JCQP)
add_subdirectory(src/osqp)

## Declare a C++ library
FILE(GLOB SRC
  src/*.cpp
  src/Controllers/*.cpp
  src/Dynamics/*.cpp
  src/MPC_Ctrl/*.cpp)
add_library(${PROJECT_NAME}
  ${SRC}
  src/GaitCtrller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  JCQP
  qpOASES
  osqp
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/walking_simulation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    config
    models
    msg
    rviz
    urdf
    worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
