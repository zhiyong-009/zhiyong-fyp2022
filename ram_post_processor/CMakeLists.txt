cmake_minimum_required(VERSION 3.2)
project(ram_post_processor)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  genmsg
  geometry_msgs
  industrial_robot_angle_conversions
  message_generation
  message_runtime
  ram_msgs
  ram_utils
  roscpp
  std_msgs
  tf2_ros
  tf_conversions
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ProgramInformation.msg
  RobotProgram.msg
  SaveOnDisk.msg
)

add_service_files(
  FILES
  GenerateCSV.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  eigen_conversions
  genmsg
  geometry_msgs
  industrial_robot_angle_conversions
  message_generation
  message_runtime
  ram_msgs
  ram_utils
  roscpp
  std_msgs
  tf2_ros
  tf_conversions
  LIBRARIES
  ${PROJECT_NAME}_csv_post_processor
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## CSV Post Processor
add_library(
  ${PROJECT_NAME}_csv_post_processor
  src/csv_post_processor.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_csv_post_processor
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_csv_post_processor
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS
  ${PROJECT_NAME}_csv_post_processor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
