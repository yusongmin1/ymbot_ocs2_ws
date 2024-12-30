cmake_minimum_required(VERSION 2.8.3)
project(ymbot_d_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  std_msgs
  std_srvs
  ymbot_devices_driver
  ocs2_msgs
  ocs2_core
  ocs2_mpc
  std_msgs
  visualization_msgs
  geometry_msgs
  interactive_markers
  moveit_core
  moveit_ros_planning  
  moveit_ros_planning_interface
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros_control_example
  CATKIN_DEPENDS controller_manager roscpp rospy std_msgs std_srvs moveit_core  moveit_ros_planning  moveit_ros_planning_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(ymbot_d_ros_control_sim src/ymbot_d_ros_control_sim.cpp) 
target_link_libraries(ymbot_d_ros_control_sim
  ${catkin_LIBRARIES} 
  ${ymbot_devices_driver_LIBRARIES}
)

add_executable(ymbot_d_ros_control_real src/ymbot_d_ros_control_real.cpp) 
target_link_libraries(ymbot_d_ros_control_real
  ${catkin_LIBRARIES} 
  ${ymbot_devices_driver_LIBRARIES}
)

add_executable(disable_all_motors src/ymbot_disable_all_motors.cpp) 
target_link_libraries(disable_all_motors
  ${catkin_LIBRARIES} 
  ${ymbot_devices_driver_LIBRARIES}
)


# Target node
add_executable(move_to_start_pose src/move_to_start_pose.cpp)
target_link_libraries(move_to_start_pose ${catkin_LIBRARIES} ${ymbot_devices_driver_LIBRARIES})