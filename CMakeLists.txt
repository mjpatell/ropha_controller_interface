cmake_minimum_required(VERSION 2.8.3)
project(ropha_controller_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# set catkin run package
SET(CATKIN_RUN_PACKAGES
  actionlib
  geometry_msgs
  roscpp
  roslib
  rospy
  std_msgs
)

# set cakin build package
SET(CATKIN_BUILD_PACKAGES
  ${CATKIN_RUN_PACKAGES}
  cmake_modules
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_PACKAGES})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ropha_controller_interface
  CATKIN_DEPENDS ${CATKIN_RUN_PACKAGES}
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

