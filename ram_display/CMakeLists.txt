cmake_minimum_required(VERSION 3.2)
project(ram_display)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_generation
  message_runtime
  ram_msgs
  ram_utils
  roscpp
  rviz
  std_msgs
  tf_conversions
)

# Check RVT minimum version
find_package(
  rviz_visual_tools
  3.6.1
  REQUIRED
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  DeleteTrajectory.srv
  DisplayRangeOfLayers.srv
  DisplayTrajectory.srv
  UpdateMeshColor.srv
  UpdateSelection.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  ram_msgs
  std_msgs
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
  ram_msgs
  ram_utils
  roscpp
  rviz
  rviz_visual_tools
  std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Implementation of the services servers
add_executable(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${rviz_visual_tools_LIBRARIES} # Manual find package
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  config
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  cmake_policy(SET CMP0046 OLD) # Ignore warning about non-existent dependency
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    ${PROJECT_NAME}_services_clients
    test/services_clients.launch
    test/services_clients.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_services_clients
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    ${PROJECT_NAME}_services_clients
    ${PROJECT_NAME}
    ${PROJECT_NAME}_generate_messages_cpp
    ram_utils_frames_tf_broadcaster # We need this node to be built
    ${catkin_EXPORTED_TARGETS}
  )
endif()
