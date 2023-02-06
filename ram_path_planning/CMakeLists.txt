cmake_minimum_required(VERSION 3.2)
project(ram_path_planning)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  eigen_conversions
  geometry_msgs
  message_generation
  message_runtime
  ram_msgs
  ram_utils
  roscpp
  roslib
  std_msgs
  tf_conversions
  unique_id
  uuid_msgs
  visualization_msgs
)

find_package(VTK 8.0.0 REQUIRED)
include(${VTK_USE_FILE})

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  FILES
  Contours.action
  DonghongDing.action
  FollowPoses.action
  PolygonOffsets.action
  Profile.action
  Revolve.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  ram_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_clipper
  ${PROJECT_NAME}_mesh_slicer
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_runtime
  ram_msgs
  ram_utils
  roscpp
  std_msgs
  uuid_msgs
  visualization_msgs
  DEPENDS
  VTK
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Angus Johnson clipper library: http://www.angusj.com/delphi/clipper.php
add_library(
  ${PROJECT_NAME}_clipper
  src/clipper/clipper.cpp
)

# Library (mostly VTK) that slices a mesh into layers
add_library(
  ${PROJECT_NAME}_mesh_slicer
  src/mesh_slicer.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_mesh_slicer
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_mesh_slicer
  ${catkin_EXPORTED_TARGETS}
)

# Service node
add_executable(
  ${PROJECT_NAME}
  src/path_planning.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${PROJECT_NAME}_clipper
  ${PROJECT_NAME}_mesh_slicer
)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_clipper
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_clipper
  ${PROJECT_NAME}_mesh_slicer
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

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  launch
  meshes # Make sure meshes are not too heavy
  svg
  yaml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    ${PROJECT_NAME}_services_clients
    test/services_clients.launch
    test/services_clients.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_services_clients
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_mesh_slicer
    ${VTK_LIBRARIES}
  )
  add_dependencies(
    ${PROJECT_NAME}_services_clients
    ${PROJECT_NAME}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
  )
endif()
