cmake_minimum_required(VERSION 2.8.3)
project(gps_extension)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nmea_navsat_driver
)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS 
		geometry_msgs 
		roscpp rospy 
		sensor_msgs 
		std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(utm_conversion src/UTM_converter.cpp)
add_dependencies(utm_conversion robot_cooperation_project_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(utm_conversion ${catkin_LIBRARIES})

#############
## Install ##
#############



#############
## Testing ##
#############

