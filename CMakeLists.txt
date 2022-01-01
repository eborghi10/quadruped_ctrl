cmake_minimum_required(VERSION 3.0.2)
project(quadruped_ctrl)

set(CMAKE_CXX_FLAGS "-std=c++14 -Werror -Wall -Wextra" CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE "Release")

find_package (Eigen3 REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    roslint
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
    geometry_msgs
    message_runtime
    roslib
    std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  src
  src/Controllers/BalanceController
  src/ParamHandler
)

add_subdirectory(src/Goldfarb_Optimizer)
add_subdirectory(src/inih)
add_subdirectory(src/ParamHandler)
add_subdirectory(src/qpOASES)
add_subdirectory(src/JCQP)
add_subdirectory(src/osqp)

## Declare a C++ library
FILE(GLOB SRC
  src/*.cpp
  src/Controllers/*.cpp
  src/Controllers/BalanceController/*.cpp
  src/Controllers/WBC_Ctrl/*.cpp
  src/Controllers/WBC_Ctrl/ContactSet/*.cpp
  src/Controllers/WBC_Ctrl/LocomotionCtrl/*.cpp
  src/Controllers/WBC_Ctrl/TaskSet/*.cpp
  src/Controllers/WBC/WBIC/*.cpp
  src/Dynamics/*.cpp
  src/FSM_States/*.cpp
  src/MPC_Ctrl/*.cpp
  src/Utilities/*.cpp
)

add_library(${PROJECT_NAME} ${SRC})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  dynacore_param_handler
  Goldfarb_Optimizer
  inih
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

# http://wiki.ros.org/roslint
# 1. Add roslint_python
# 2. Make build fail with roslint_add_test()
# 3. Set custom options and apply some fixes automatically: http://wiki.ros.org/roslint#Tips
# roslint_cpp(${SRC})
