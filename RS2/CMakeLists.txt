cmake_minimum_required(VERSION 3.0.2)
project(RS2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_commander
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES RS2
  CATKIN_DEPENDS roscpp rospy std_msgs moveit_commander
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/Aruco_detection_usb.py
  scripts/Get_final_points_to_robot.py
  scripts/Paper_Robot_Frame.py
  scripts/PixelMM.py
  scripts/Run_Final.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
