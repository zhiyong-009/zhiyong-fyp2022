cmake_minimum_required(VERSION 3.2)
project(ram_msgs)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
  uuid_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  AdditiveManufacturingParams.msg
  AdditiveManufacturingPose.msg
  AdditiveManufacturingTrajectory.msg
  AdditiveManufacturingTrajectoryInfo.msg
  FillParameters.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  uuid_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  roscpp
  std_msgs
  uuid_msgs
)

#############
## Install ##
#############

# Messages are installed automatically by catkin

#############
## Testing ##
#############
